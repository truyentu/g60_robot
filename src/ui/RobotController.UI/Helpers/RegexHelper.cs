using System.Text.RegularExpressions;
using RobotController.UI.Models;

namespace RobotController.UI.Helpers;

/// <summary>
/// Regex patterns and helpers for parsing/replacing RPL (Robot Programming Language) constructs.
/// Supports ABB RAPID-like syntax used in the Program Editor.
/// </summary>
public static partial class RegexHelper
{
    // ========================================================================
    // Motion Instruction Patterns
    // ========================================================================

    /// <summary>
    /// Match a motion instruction line: MoveL/MoveJ/MoveC targetName, speedName, zoneName, toolName;
    /// Groups: [1]=MotionType, [2]=TargetName, [3]=SpeedName, [4]=ZoneName, [5]=ToolName
    /// </summary>
    [GeneratedRegex(@"(Move[LJC])\s+([a-zA-Z_]\w*)\s*,\s*([a-zA-Z_]\w*)\s*,\s*([a-zA-Z_]\w*)\s*,\s*([a-zA-Z_]\w*)\s*;?",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex MotionInstructionRegex();

    /// <summary>
    /// Simple match for any motion command to extract the target variable name.
    /// Groups: [1]=MotionType (MoveL/MoveJ/MoveC), [2]=TargetName
    /// </summary>
    [GeneratedRegex(@"(Move[LJC])\s+([a-zA-Z_]\w*)\b", RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex MotionTargetRegex();

    // ========================================================================
    // Variable Definition Patterns
    // ========================================================================

    /// <summary>
    /// Match robtarget variable definition and capture the value part.
    /// Pattern: (CONST|VAR|PERS)? robtarget varName := [[x,y,z],[q1,q2,q3,q4],[cf],[ext]];
    /// Groups: [1]=Qualifier(opt), [2]=VarName, [3]=Value (everything inside outer brackets)
    /// </summary>
    [GeneratedRegex(@"(?:CONST|VAR|PERS)?\s*robtarget\s+([a-zA-Z_]\w*)\s*:=\s*(\[\[[\s\S]*?\]\])\s*;?",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex RobTargetDefinitionRegex();

    /// <summary>
    /// Match just the position part from a robtarget value: [[x,y,z],...]
    /// Groups: [1]=X, [2]=Y, [3]=Z
    /// </summary>
    [GeneratedRegex(@"\[\[\s*([\d\.\-eE]+)\s*,\s*([\d\.\-eE]+)\s*,\s*([\d\.\-eE]+)\s*\]",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex PositionRegex();

    /// <summary>
    /// Match quaternion part from robtarget: [q1,q2,q3,q4]  (second bracket group)
    /// Used after extracting position part.
    /// Groups: [1]=Q1, [2]=Q2, [3]=Q3, [4]=Q4
    /// </summary>
    [GeneratedRegex(@"\]\s*,\s*\[\s*([\d\.\-eE]+)\s*,\s*([\d\.\-eE]+)\s*,\s*([\d\.\-eE]+)\s*,\s*([\d\.\-eE]+)\s*\]",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex QuaternionRegex();

    // ========================================================================
    // Process Command Patterns
    // ========================================================================

    /// <summary>Match ArcStart with optional Job_ID parameter</summary>
    [GeneratedRegex(@"ArcStart\s*(?:\(\s*Job_?ID\s*:=\s*(\d+)\s*\))?",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex ArcStartRegex();

    /// <summary>Match ArcEnd</summary>
    [GeneratedRegex(@"ArcEnd\b", RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex ArcEndRegex();

    // ========================================================================
    // Helper Methods
    // ========================================================================

    /// <summary>
    /// Extract the target variable name from a motion instruction line.
    /// Returns null if the line is not a motion instruction.
    /// </summary>
    public static string? ExtractTargetName(string lineText)
    {
        var match = MotionTargetRegex().Match(lineText);
        return match.Success ? match.Groups[2].Value : null;
    }

    /// <summary>
    /// Extract the motion type from a motion instruction line.
    /// </summary>
    public static MotionType? ExtractMotionType(string lineText)
    {
        var match = MotionTargetRegex().Match(lineText);
        if (!match.Success) return null;

        return match.Groups[1].Value.ToUpperInvariant() switch
        {
            "MOVEL" => MotionType.MoveL,
            "MOVEJ" => MotionType.MoveJ,
            "MOVEC" => MotionType.MoveC,
            _ => null
        };
    }

    /// <summary>
    /// Build a regex to find a specific variable's definition in the document.
    /// Searches for: (CONST|VAR|PERS)? robtarget {varName} := {value};
    /// Returns a regex with groups: [1]=prefix (everything before value), [2]=value (bracket content)
    /// </summary>
    public static Regex BuildDefinitionFinder(string variableName)
    {
        var escaped = Regex.Escape(variableName);
        var pattern = $@"((?:CONST|VAR|PERS\s+)?robtarget\s+{escaped}\s*:=\s*)(\[\[[\s\S]*?\]\])";
        return new Regex(pattern, RegexOptions.IgnoreCase);
    }

    /// <summary>
    /// Find the definition of a variable in the full document text.
    /// Returns (startOffset, length) of just the VALUE part (the bracket content).
    /// Returns null if not found.
    /// </summary>
    public static (int offset, int length, string currentValue)? FindVariableDefinition(
        string documentText, string variableName)
    {
        var regex = BuildDefinitionFinder(variableName);
        var match = regex.Match(documentText);
        if (!match.Success) return null;

        // Group[2] is the value part [[...]]
        return (match.Groups[2].Index, match.Groups[2].Length, match.Groups[2].Value);
    }

    /// <summary>
    /// Parse position (X, Y, Z) from a robtarget value string like [[100,200,300],[1,0,0,0],...]
    /// </summary>
    public static (double x, double y, double z)? ParsePosition(string robTargetValue)
    {
        var match = PositionRegex().Match(robTargetValue);
        if (!match.Success) return null;

        if (double.TryParse(match.Groups[1].Value, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var x) &&
            double.TryParse(match.Groups[2].Value, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var y) &&
            double.TryParse(match.Groups[3].Value, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var z))
        {
            return (x, y, z);
        }

        return null;
    }

    /// <summary>
    /// Build a coordinate cache from document text.
    /// Returns Dictionary mapping variable name -> (x, y, z).
    /// </summary>
    public static Dictionary<string, (double x, double y, double z)> BuildCoordinateCache(string documentText)
    {
        var cache = new Dictionary<string, (double, double, double)>(StringComparer.OrdinalIgnoreCase);
        var matches = RobTargetDefinitionRegex().Matches(documentText);

        foreach (Match m in matches)
        {
            var varName = m.Groups[1].Value;
            var value = m.Groups[2].Value;
            var pos = ParsePosition(value);
            if (pos.HasValue)
            {
                cache[varName] = pos.Value;
            }
        }

        return cache;
    }

    /// <summary>
    /// Check if a line is a motion instruction (MoveL, MoveJ, MoveC)
    /// </summary>
    public static bool IsMotionInstruction(string lineText)
    {
        return MotionTargetRegex().IsMatch(lineText);
    }
}
