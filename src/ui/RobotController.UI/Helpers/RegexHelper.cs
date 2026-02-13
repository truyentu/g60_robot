using System.Text.RegularExpressions;
using RobotController.UI.Models;

namespace RobotController.UI.Helpers;

/// <summary>
/// Regex patterns and helpers for parsing/replacing KRL (KUKA Robot Language) constructs.
/// Supports KRL syntax used in the Program Editor.
/// </summary>
public static partial class RegexHelper
{
    // ========================================================================
    // Motion Instruction Patterns (KRL)
    // ========================================================================

    /// <summary>
    /// Match a KRL motion instruction line: PTP/LIN/CIRC targetName [C_PTP|C_DIS|C_VEL|C_ORI]
    /// Groups: [1]=MotionType, [2]=TargetName
    /// </summary>
    [GeneratedRegex(@"(PTP|LIN|CIRC|PTP_REL|LIN_REL|CIRC_REL)\s+([a-zA-Z_]\w*)\b",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex MotionTargetRegex();

    /// <summary>
    /// Match a full KRL motion instruction with optional approximation:
    /// PTP/LIN/CIRC target [C_PTP|C_DIS|C_VEL|C_ORI]
    /// Groups: [1]=MotionType, [2]=TargetName, [3]=Approximation (optional)
    /// </summary>
    [GeneratedRegex(@"(PTP|LIN|CIRC|PTP_REL|LIN_REL|CIRC_REL)\s+([a-zA-Z_]\w*)(?:\s+(C_PTP|C_DIS|C_VEL|C_ORI))?\s*$",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex KrlMotionRegex();

    /// <summary>
    /// Match a KRL system variable assignment: $VEL.CP = 0.1 or $APO.CDIS = 10 etc.
    /// Groups: [1]=VarName, [2]=SubField (optional), [3]=Value
    /// </summary>
    [GeneratedRegex(@"\$(\w+)(?:\.(\w+))?\s*=\s*([\d\.\-eE]+)",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex SystemVarAssignRegex();

    /// <summary>
    /// Match $TOOL = TOOL_DATA[N] or $BASE = BASE_DATA[N]
    /// Groups: [1]=TOOL or BASE, [2]=Index
    /// </summary>
    [GeneratedRegex(@"\$(TOOL|BASE)\s*=\s*(?:TOOL_DATA|BASE_DATA)\s*\[\s*(\d+)\s*\]",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex ToolBaseAssignRegex();

    // ========================================================================
    // Variable Definition Patterns (KRL)
    // ========================================================================

    /// <summary>
    /// Match E6POS/POS variable definition and capture the value part.
    /// Pattern: DECL E6POS varName = {X ..., Y ..., Z ..., A ..., B ..., C ...}
    /// Also matches: CONST E6POS varName = {X ..., Y ..., Z ..., A ..., B ..., C ...}
    /// Groups: [1]=VarName, [2]=Value (the aggregate {...})
    /// </summary>
    [GeneratedRegex(@"(?:DECL|CONST)\s+(?:E6POS|POS|FRAME)\s+([a-zA-Z_]\w*)\s*=\s*(\{[\s\S]*?\})",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex E6PosDefinitionRegex();

    /// <summary>
    /// Extract X, Y, Z from KRL aggregate: {X val, Y val, Z val, A val, B val, C val}
    /// Groups: [1]=X, [2]=Y, [3]=Z
    /// </summary>
    [GeneratedRegex(@"X\s+([\d\.\-eE]+)\s*,\s*Y\s+([\d\.\-eE]+)\s*,\s*Z\s+([\d\.\-eE]+)",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex PositionRegex();

    /// <summary>
    /// Extract A, B, C from KRL aggregate: {X val, Y val, Z val, A val, B val, C val}
    /// Groups: [1]=A, [2]=B, [3]=C
    /// </summary>
    [GeneratedRegex(@"A\s+([\d\.\-eE]+)\s*,\s*B\s+([\d\.\-eE]+)\s*,\s*C\s+([\d\.\-eE]+)",
        RegexOptions.IgnoreCase | RegexOptions.Compiled)]
    public static partial Regex OrientationRegex();

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
    /// Extract the target variable name from a KRL motion instruction line.
    /// Returns null if the line is not a motion instruction.
    /// </summary>
    public static string? ExtractTargetName(string lineText)
    {
        var match = MotionTargetRegex().Match(lineText);
        return match.Success ? match.Groups[2].Value : null;
    }

    /// <summary>
    /// Extract approximation type from a KRL motion line.
    /// Returns EXACT if no approximation keyword found.
    /// </summary>
    public static ApproximationType ExtractApproximation(string lineText)
    {
        var match = KrlMotionRegex().Match(lineText);
        if (!match.Success || !match.Groups[3].Success) return ApproximationType.EXACT;

        return match.Groups[3].Value.ToUpperInvariant() switch
        {
            "C_PTP" => ApproximationType.C_PTP,
            "C_DIS" => ApproximationType.C_DIS,
            "C_VEL" => ApproximationType.C_VEL,
            "C_ORI" => ApproximationType.C_ORI,
            _ => ApproximationType.EXACT
        };
    }

    /// <summary>
    /// Extract the motion type from a KRL motion instruction line.
    /// </summary>
    public static MotionType? ExtractMotionType(string lineText)
    {
        var match = MotionTargetRegex().Match(lineText);
        if (!match.Success) return null;

        return match.Groups[1].Value.ToUpperInvariant() switch
        {
            "PTP" or "PTP_REL" => MotionType.PTP,
            "LIN" or "LIN_REL" => MotionType.LIN,
            "CIRC" or "CIRC_REL" => MotionType.CIRC,
            _ => null
        };
    }

    /// <summary>
    /// Build a regex to find a specific variable's KRL definition in the document.
    /// Searches for: (DECL|CONST) (E6POS|POS|FRAME) {varName} = {value}
    /// Returns a regex with groups: [1]=prefix (everything before value), [2]=value (aggregate content)
    /// </summary>
    public static Regex BuildDefinitionFinder(string variableName)
    {
        var escaped = Regex.Escape(variableName);
        var pattern = $@"((?:DECL|CONST)\s+(?:E6POS|POS|FRAME)\s+{escaped}\s*=\s*)(\{{[\s\S]*?\}})";
        return new Regex(pattern, RegexOptions.IgnoreCase);
    }

    /// <summary>
    /// Find the definition of a variable in the full document text.
    /// Returns (startOffset, length) of just the VALUE part (the aggregate content).
    /// Returns null if not found.
    /// </summary>
    public static (int offset, int length, string currentValue)? FindVariableDefinition(
        string documentText, string variableName)
    {
        var regex = BuildDefinitionFinder(variableName);
        var match = regex.Match(documentText);
        if (!match.Success) return null;

        // Group[2] is the value part {...}
        return (match.Groups[2].Index, match.Groups[2].Length, match.Groups[2].Value);
    }

    /// <summary>
    /// Parse position (X, Y, Z) from a KRL E6POS aggregate string like {X 100, Y 200, Z 300, A 0, B 0, C 0}
    /// </summary>
    public static (double x, double y, double z)? ParsePosition(string e6posValue)
    {
        var match = PositionRegex().Match(e6posValue);
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
        var matches = E6PosDefinitionRegex().Matches(documentText);

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
    /// Check if a line is a KRL motion instruction (PTP, LIN, CIRC)
    /// </summary>
    public static bool IsMotionInstruction(string lineText)
    {
        return MotionTargetRegex().IsMatch(lineText);
    }
}
