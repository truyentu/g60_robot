using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using RobotController.UI.Models;

namespace RobotController.UI.Services;

/// <summary>
/// PSO particle for spin angle optimization.
/// Each particle represents a complete set of spin angles for all path points.
/// </summary>
internal class Particle
{
    public double[] Position { get; set; }
    public double[] Velocity { get; set; }
    public double[] BestPosition { get; set; }
    public double BestCost { get; set; } = double.MaxValue;

    public Particle(int dimensions, double minVal, double maxVal)
    {
        Position = new double[dimensions];
        Velocity = new double[dimensions];
        BestPosition = new double[dimensions];

        for (int i = 0; i < dimensions; i++)
        {
            Position[i] = Random.Shared.NextDouble() * (maxVal - minVal) + minVal;
            Velocity[i] = (Random.Shared.NextDouble() - 0.5) * (maxVal - minVal) * 0.1;
            BestPosition[i] = Position[i];
        }
    }
}

/// <summary>
/// Interface for IK computation proxy (abstracts C++ core or C# analytical IK).
/// </summary>
public interface IKinematicsProxy
{
    /// <summary>
    /// Solve IK for a 6-DOF pose. Returns joint angles in radians, or null if infeasible.
    /// </summary>
    Task<double[]?> SolveIKAsync(double[] pose6d);

    /// <summary>
    /// Joint limits [min, max] in radians for each of 6 joints.
    /// </summary>
    (double min, double max)[] JointLimits { get; }
}

/// <summary>
/// Modified Particle Swarm Optimization (MPSO) for spin angle optimization.
///
/// Based on: Doan et al. "Optimal robot placement for minimizing wrist singularity"
///
/// Optimizes spin angles φ[i] ∈ [-180°, 180°] for each path waypoint to:
///   1. Maximize distance from wrist singularity (θ₅ away from 0)
///   2. Stay within joint limits
///   3. Maintain smooth joint trajectories
/// </summary>
public class SpinAngleOptimizer
{
    // PSO hyperparameters (from Doan paper, Table 2)
    private const int SwarmSize = 30;
    private const int MaxIterations = 100;
    private const double InertiaWeightStart = 0.9;
    private const double InertiaWeightEnd = 0.4;
    private const double C1 = 1.5;  // Cognitive (personal best attraction)
    private const double C2 = 1.5;  // Social (global best attraction)
    private const double MinSpinAngle = -180.0;
    private const double MaxSpinAngle = 180.0;

    // Cost function weights
    private const double SingularityWeight = 10.0;
    private const double JointLimitWeight = 50.0;
    private const double SmoothnessWeight = 0.5;
    private const double JointLimitWarningRatio = 0.85;

    /// <summary>
    /// Progress callback: (iteration, bestCost, percentComplete)
    /// </summary>
    public event Action<int, double, double>? ProgressChanged;

    /// <summary>
    /// Run MPSO optimization for a weld path's spin angles.
    /// </summary>
    public async Task<SpinAngleOptimizationResult> OptimizeAsync(
        WeldPath path,
        IKinematicsProxy kinProxy,
        CancellationToken ct = default)
    {
        int N = path.Points.Count;
        if (N < 2) throw new ArgumentException("Path must have at least 2 points");

        var sw = Stopwatch.StartNew();

        // Evaluate initial cost (before optimization)
        double[] initialSpins = new double[N];
        for (int i = 0; i < N; i++)
            initialSpins[i] = path.Points[i].SpinAngle;
        double initialCost = await EvaluateCostAsync(initialSpins, path, kinProxy);
        double minTheta5Before = FindMinTheta5(path);

        // Initialize swarm
        var swarm = new Particle[SwarmSize];
        for (int p = 0; p < SwarmSize; p++)
        {
            swarm[p] = new Particle(N, MinSpinAngle, MaxSpinAngle);
            // Seed first particle with current spin angles
            if (p == 0)
            {
                for (int i = 0; i < N; i++)
                    swarm[0].Position[i] = path.Points[i].SpinAngle;
            }
        }

        double globalBestCost = double.MaxValue;
        double[] globalBest = new double[N];

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            ct.ThrowIfCancellationRequested();

            // Adaptive inertia weight (linearly decreasing)
            double w = InertiaWeightStart -
                       (InertiaWeightStart - InertiaWeightEnd) * iter / MaxIterations;

            // Evaluate all particles
            var tasks = new Task<double>[SwarmSize];
            for (int p = 0; p < SwarmSize; p++)
            {
                int pi = p;
                tasks[pi] = EvaluateCostAsync(swarm[pi].Position, path, kinProxy);
            }
            await Task.WhenAll(tasks);

            for (int p = 0; p < SwarmSize; p++)
            {
                double cost = tasks[p].Result;

                // Update personal best
                if (cost < swarm[p].BestCost)
                {
                    swarm[p].BestCost = cost;
                    Array.Copy(swarm[p].Position, swarm[p].BestPosition, N);
                }

                // Update global best
                if (cost < globalBestCost)
                {
                    globalBestCost = cost;
                    Array.Copy(swarm[p].Position, globalBest, N);
                }
            }

            // Update velocities and positions
            for (int p = 0; p < SwarmSize; p++)
            {
                for (int d = 0; d < N; d++)
                {
                    double r1 = Random.Shared.NextDouble();
                    double r2 = Random.Shared.NextDouble();

                    swarm[p].Velocity[d] = w * swarm[p].Velocity[d]
                        + C1 * r1 * (swarm[p].BestPosition[d] - swarm[p].Position[d])
                        + C2 * r2 * (globalBest[d] - swarm[p].Position[d]);

                    // Velocity clamping
                    double maxVel = (MaxSpinAngle - MinSpinAngle) * 0.2;
                    swarm[p].Velocity[d] = Math.Clamp(swarm[p].Velocity[d], -maxVel, maxVel);

                    swarm[p].Position[d] += swarm[p].Velocity[d];
                    swarm[p].Position[d] = Math.Clamp(swarm[p].Position[d], MinSpinAngle, MaxSpinAngle);
                }
            }

            ProgressChanged?.Invoke(iter + 1, globalBestCost,
                (double)(iter + 1) / MaxIterations * 100.0);
        }

        // Apply optimized spin angles to path
        for (int i = 0; i < N; i++)
            path.Points[i].SpinAngle = globalBest[i];

        sw.Stop();

        return new SpinAngleOptimizationResult
        {
            OptimizedSpinAngles = globalBest,
            InitialCost = initialCost,
            FinalCost = globalBestCost,
            Iterations = MaxIterations,
            MinTheta5Before = minTheta5Before,
            MinTheta5After = 0, // Will be computed after re-evaluation
            ElapsedTime = sw.Elapsed
        };
    }

    /// <summary>
    /// Evaluate cost function for a set of spin angles.
    /// </summary>
    private async Task<double> EvaluateCostAsync(
        double[] spinAngles, WeldPath path, IKinematicsProxy kin)
    {
        double cost = 0;
        double[]? prevJoints = null;
        var limits = kin.JointLimits;

        for (int i = 0; i < path.Points.Count; i++)
        {
            // Build 6-DOF pose from 5-DOF task + spin angle
            double[] pose = BuildPoseWithSpin(path.Points[i], spinAngles[i]);
            double[]? joints = await kin.SolveIKAsync(pose);

            if (joints == null)
                return double.MaxValue; // Infeasible — no IK solution

            // 1. Singularity penalty: maximize |θ₅|
            // θ₅ is joint index 4 (0-based)
            double theta5Abs = Math.Abs(joints[4]);
            cost += SingularityWeight / (theta5Abs + 0.01);

            // 2. Joint limit penalty (quartic barrier near limits)
            for (int j = 0; j < 6; j++)
            {
                double mid = (limits[j].max + limits[j].min) / 2.0;
                double halfRange = (limits[j].max - limits[j].min) / 2.0;
                if (halfRange < 1e-6) continue;

                double normalizedDist = Math.Abs(joints[j] - mid) / halfRange;
                if (normalizedDist > JointLimitWarningRatio)
                    cost += JointLimitWeight * Math.Pow(normalizedDist, 4);
            }

            // 3. Smoothness penalty (minimize joint velocity between waypoints)
            if (prevJoints != null)
            {
                for (int j = 0; j < 6; j++)
                    cost += SmoothnessWeight * Math.Pow(joints[j] - prevJoints[j], 2);
            }

            prevJoints = joints;
        }

        return cost;
    }

    /// <summary>
    /// Build a 6-DOF pose from a WeldPathPoint + spin angle.
    /// Returns [x, y, z, rx, ry, rz] in mm/radians.
    /// </summary>
    private static double[] BuildPoseWithSpin(WeldPathPoint point, double spinAngleDeg)
    {
        double spinRad = spinAngleDeg * Math.PI / 180.0;
        double workRad = point.WorkAngle * Math.PI / 180.0;
        double travelRad = point.TravelAngle * Math.PI / 180.0;

        // Torch direction from work + travel angles:
        // Work angle = rotation around seam X-axis (tilt toward workpiece)
        // Travel angle = rotation around seam Y-axis (push/drag)
        // Spin angle = rotation around torch Z-axis (redundant DOF)
        //
        // For now, encode as RPY: rz=spin, ry=travel, rx=work
        // This is a simplification — real implementation should use rotation matrix
        return new double[]
        {
            point.X, point.Y, point.Z,
            workRad, travelRad, spinRad
        };
    }

    private static double FindMinTheta5(WeldPath path)
    {
        double minTheta5 = double.MaxValue;
        foreach (var pt in path.Points)
        {
            if (pt.JointAngles != null && pt.JointAngles.Length >= 5)
            {
                double t5 = Math.Abs(pt.JointAngles[4]) * 180.0 / Math.PI;
                if (t5 < minTheta5) minTheta5 = t5;
            }
        }
        return minTheta5 == double.MaxValue ? 0 : minTheta5;
    }
}
