using System;
using System.Threading;
using System.Threading.Tasks;
using RobotController.Common.Services;

namespace RobotController.UI.Services;

/// <summary>
/// IKinematicsProxy implementation that routes IK calls through ZeroMQ IPC
/// to the C++ Core's KDL solver. Handles unit conversion between
/// MPSO (mm/radians) and IPC (mm/degrees).
/// </summary>
public class IpcKinematicsProxy : IKinematicsProxy
{
    private readonly IIpcClientService _ipcClient;
    private readonly (double min, double max)[] _jointLimits;
    private double[] _lastJointsDeg = new double[6];

    private const double Deg2Rad = Math.PI / 180.0;
    private const double Rad2Deg = 180.0 / Math.PI;

    public IpcKinematicsProxy(
        IIpcClientService ipcClient,
        (double min, double max)[] jointLimitsRadians,
        double[]? initialSeedDeg = null)
    {
        _ipcClient = ipcClient;
        _jointLimits = jointLimitsRadians;
        if (initialSeedDeg != null)
            Array.Copy(initialSeedDeg, _lastJointsDeg, Math.Min(initialSeedDeg.Length, 6));
    }

    public (double min, double max)[] JointLimits => _jointLimits;

    public async Task<double[]?> SolveIKAsync(double[] pose6d)
    {
        // pose6d from MPSO: [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
        // IPC expects:      [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        var targetPoseDeg = new double[]
        {
            pose6d[0], pose6d[1], pose6d[2],
            pose6d[3] * Rad2Deg, pose6d[4] * Rad2Deg, pose6d[5] * Rad2Deg
        };

        var response = await _ipcClient.ComputeIKAsync(
            targetPoseDeg, _lastJointsDeg, apply: false);

        if (response == null || !response.Success)
            return null;

        // IPC returns joints in DEGREES, MPSO needs RADIANS
        var jointsRad = new double[6];
        for (int i = 0; i < 6; i++)
        {
            jointsRad[i] = response.Joints[i] * Deg2Rad;
            _lastJointsDeg[i] = response.Joints[i];
        }

        return jointsRad;
    }
}
