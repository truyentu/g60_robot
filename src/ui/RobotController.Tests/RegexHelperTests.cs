using RobotController.UI.Helpers;
using RobotController.UI.Models;

namespace RobotController.Tests;

/// <summary>
/// Unit tests for RegexHelper - verifying robustness with messy inputs.
/// Covers: tabs, extra spaces, trailing comments, mixed case, malformed syntax.
/// Tests updated for pure KRL (KUKA Robot Language) syntax.
/// </summary>
public class RegexHelperTests
{
    // ========================================================================
    // ExtractTargetName - Basic Cases (KRL)
    // ========================================================================

    [Theory]
    [InlineData("LIN p1", "p1")]
    [InlineData("PTP pHome", "pHome")]
    [InlineData("CIRC pArc", "pArc")]
    [InlineData("PTP_REL pRel", "pRel")]
    [InlineData("LIN_REL pRel2", "pRel2")]
    public void ExtractTargetName_StandardInput_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // ExtractTargetName - Messy Inputs
    // ========================================================================

    [Theory]
    [InlineData("    LIN p1", "p1")]                              // Leading spaces
    [InlineData("\tPTP p1", "p1")]                                // Leading tab
    [InlineData("\t\t  LIN p1", "p1")]                            // Tabs + spaces
    [InlineData("LIN   p1", "p1")]                                // Extra spaces
    [InlineData("LIN\tp1", "p1")]                                 // Tab as separator
    [InlineData("LIN  \t p1", "p1")]                              // Mixed tabs+spaces
    public void ExtractTargetName_MessyWhitespace_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("LIN P1", "P1")]                                  // Uppercase target
    [InlineData("lin p1", "p1")]                                  // Lowercase motion
    [InlineData("PTP P1", "P1")]                                  // All uppercase
    [InlineData("Lin p1", "p1")]                                  // Mixed case motion
    [InlineData("LIN my_target_1", "my_target_1")]                // Underscore in name
    public void ExtractTargetName_CaseVariations_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("LIN p1 ; go to start", "p1")]                   // KRL comment after
    [InlineData("  PTP p1 C_PTP", "p1")]                          // With approximation
    public void ExtractTargetName_TrailingContent_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("; This is a KRL comment", null)]                  // Comment-only line
    [InlineData("", null)]                                         // Empty line
    [InlineData("   ", null)]                                      // Whitespace-only
    [InlineData("ArcStart(Job_ID:=1)", null)]                      // Process command
    [InlineData("ArcEnd", null)]                                   // Process command
    [InlineData("DEF WeldProgram()", null)]                        // Routine definition
    [InlineData("END", null)]                                      // End block
    [InlineData("DECL E6POS p1 = {X 100, Y 200, Z 300, A 0, B 0, C 0}", null)] // Variable definition (not motion)
    [InlineData("IF flag == TRUE THEN", null)]                     // Control flow
    public void ExtractTargetName_NonMotionLines_ReturnsNull(string line, string? expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // ExtractMotionType (KRL)
    // ========================================================================

    [Theory]
    [InlineData("LIN p1", MotionType.LIN)]
    [InlineData("PTP pHome", MotionType.PTP)]
    [InlineData("CIRC pArc", MotionType.CIRC)]
    [InlineData("PTP_REL pRel", MotionType.PTP)]
    [InlineData("LIN_REL pRel", MotionType.LIN)]
    [InlineData("\t  lin  p1", MotionType.LIN)]                   // Messy + lowercase
    [InlineData("; comment line", null)]
    [InlineData("ArcStart", null)]
    public void ExtractMotionType_VariousInputs_ReturnsCorrectType(string line, MotionType? expected)
    {
        var result = RegexHelper.ExtractMotionType(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // IsMotionInstruction (KRL)
    // ========================================================================

    [Theory]
    [InlineData("LIN p1", true)]
    [InlineData("\tPTP pHome ; comment", true)]
    [InlineData("; just a comment", false)]
    [InlineData("ArcStart(Job_ID:=1)", false)]
    [InlineData("DECL E6POS p1 = {X 100, Y 0, Z 0, A 0, B 0, C 0}", false)]
    [InlineData("", false)]
    public void IsMotionInstruction_VariousInputs_ReturnsCorrectResult(string line, bool expected)
    {
        var result = RegexHelper.IsMotionInstruction(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // FindVariableDefinition - Core Touch-Up logic (KRL)
    // ========================================================================

    [Fact]
    public void FindVariableDefinition_StandardDefinition_FindsValueRange()
    {
        var doc = "DECL E6POS p1 = {X 600.00, Y 100.00, Z 500.00, A 0.00, B 90.00, C 0.00}";
        var result = RegexHelper.FindVariableDefinition(doc, "p1");

        Assert.NotNull(result);
        var (offset, length, value) = result.Value;
        Assert.StartsWith("{X 600.00", value);
        Assert.EndsWith("}", value);
        Assert.Equal(value, doc.Substring(offset, length));
    }

    [Fact]
    public void FindVariableDefinition_WithIndentation_PreservesContext()
    {
        var doc = @"; Program header
    DECL E6POS p1 = {X 100.00, Y 200.00, Z 300.00, A 0.00, B 0.00, C 0.00}
    DECL E6POS p2 = {X 400.00, Y 500.00, Z 600.00, A 0.00, B 0.00, C 0.00}";

        var result = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(result);
        var (offset, length, value) = result.Value;

        Assert.StartsWith("{X 400.00", value);
        Assert.DoesNotContain("DECL", value);
        Assert.DoesNotContain("E6POS", value);
    }

    [Fact]
    public void FindVariableDefinition_ConstQualifier_Works()
    {
        var doc = "CONST E6POS pFixed = {X 0, Y 0, Z 0, A 0, B 0, C 0}";
        var result = RegexHelper.FindVariableDefinition(doc, "pFixed");
        Assert.NotNull(result);
        Assert.StartsWith("{X 0", result.Value.currentValue);
    }

    [Fact]
    public void FindVariableDefinition_PosType_Works()
    {
        var doc = "DECL POS pMid = {X 10, Y 20, Z 30, A 0, B 0, C 0}";
        var result = RegexHelper.FindVariableDefinition(doc, "pMid");
        Assert.NotNull(result);
    }

    [Fact]
    public void FindVariableDefinition_FrameType_Works()
    {
        var doc = "DECL FRAME base1 = {X 0, Y 0, Z 0, A 0, B 0, C 0}";
        var result = RegexHelper.FindVariableDefinition(doc, "base1");
        Assert.NotNull(result);
    }

    [Fact]
    public void FindVariableDefinition_NotFound_ReturnsNull()
    {
        var doc = "DECL E6POS p1 = {X 100, Y 200, Z 300, A 0, B 0, C 0}";
        var result = RegexHelper.FindVariableDefinition(doc, "pNotExist");
        Assert.Null(result);
    }

    [Fact]
    public void FindVariableDefinition_CaseInsensitive_MatchesTarget()
    {
        var doc = "DECL E6POS pHome = {X 500, Y 0, Z 800, A 0, B 0, C 0}";
        var result = RegexHelper.FindVariableDefinition(doc, "PHOME");
        Assert.NotNull(result);
    }

    // ========================================================================
    // FindVariableDefinition - Formatting Preservation
    // ========================================================================

    [Fact]
    public void TouchUp_Simulation_PreservesDocumentFormatting()
    {
        var originalDoc = @"; Robot Program - Welding Example
; Variable Definitions
    DECL E6POS pHome = {X 500.00, Y 0.00, Z 800.00, A 0.00, B 0.00, C 0.00}
    DECL E6POS p1 = {X 600.00, Y 100.00, Z 500.00, A 0.00, B 90.00, C 0.00}

DEF WeldProgram()
    PTP pHome
    LIN p1
END";

        // Step 1: Find p1's value range
        var definition = RegexHelper.FindVariableDefinition(originalDoc, "p1");
        Assert.NotNull(definition);
        var (offset, length, oldValue) = definition.Value;

        // Step 2: Simulate new value from Touch-Up
        var newValue = "{X 650.00, Y 150.00, Z 480.00, A 0.00, B 90.00, C 0.00}";

        // Step 3: Replace
        var newDoc = originalDoc.Remove(offset, length).Insert(offset, newValue);

        // VERIFY: Indentation preserved
        Assert.Contains("    DECL E6POS p1 = {X 650.00", newDoc);

        // VERIFY: Other lines untouched
        Assert.Contains("    DECL E6POS pHome = {X 500.00", newDoc);
        Assert.Contains("DEF WeldProgram()", newDoc);
        Assert.Contains("    PTP pHome", newDoc);
        Assert.Contains("    LIN p1", newDoc);
        Assert.Contains("END", newDoc);

        // VERIFY: Comment preserved
        Assert.Contains("; Robot Program - Welding Example", newDoc);

        // VERIFY: pHome definition NOT changed
        var pHomeResult = RegexHelper.FindVariableDefinition(newDoc, "pHome");
        Assert.NotNull(pHomeResult);
        Assert.Contains("500.00", pHomeResult.Value.currentValue);
    }

    [Fact]
    public void TouchUp_Simulation_MultipleUpdates_PreservesStructure()
    {
        var doc = @"DECL E6POS p1 = {X 100, Y 0, Z 0, A 0, B 0, C 0}
DECL E6POS p2 = {X 200, Y 0, Z 0, A 0, B 0, C 0}
DECL E6POS p3 = {X 300, Y 0, Z 0, A 0, B 0, C 0}";

        // Update p2
        var def = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(def);
        var newVal = "{X 250, Y 50, Z 10, A 0, B 0, C 0}";
        doc = doc.Remove(def.Value.offset, def.Value.length).Insert(def.Value.offset, newVal);

        // Verify p1 and p3 untouched
        var p1 = RegexHelper.FindVariableDefinition(doc, "p1");
        var p3 = RegexHelper.FindVariableDefinition(doc, "p3");
        Assert.NotNull(p1);
        Assert.NotNull(p3);
        Assert.Contains("X 100", p1.Value.currentValue);
        Assert.Contains("X 300", p3.Value.currentValue);

        // Verify p2 updated
        var p2 = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(p2);
        Assert.Contains("X 250", p2.Value.currentValue);
    }

    // ========================================================================
    // BuildCoordinateCache (KRL)
    // ========================================================================

    [Fact]
    public void BuildCoordinateCache_MultipleTargets_ReturnsAll()
    {
        var doc = @"DECL E6POS p1 = {X 100.0, Y 200.0, Z 300.0, A 0, B 0, C 0}
DECL E6POS p2 = {X 400.0, Y 500.0, Z 600.0, A 0, B 0, C 0}
CONST E6POS p3 = {X 700.0, Y 800.0, Z 900.0, A 0, B 0, C 0}";

        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.Equal(3, cache.Count);
        Assert.Equal(100.0, cache["p1"].x, 0.01);
        Assert.Equal(200.0, cache["p1"].y, 0.01);
        Assert.Equal(300.0, cache["p1"].z, 0.01);
        Assert.Equal(400.0, cache["p2"].x, 0.01);
        Assert.Equal(700.0, cache["p3"].x, 0.01);
    }

    [Fact]
    public void BuildCoordinateCache_ScientificNotation_ParsesCorrectly()
    {
        var doc = "DECL E6POS p1 = {X 1.5E2, Y 2.0E1, Z 3.0E-1, A 0, B 0, C 0}";
        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.Single(cache);
        Assert.Equal(150.0, cache["p1"].x, 0.01);
        Assert.Equal(20.0, cache["p1"].y, 0.01);
        Assert.Equal(0.3, cache["p1"].z, 0.01);
    }

    [Fact]
    public void BuildCoordinateCache_NegativeCoordinates_ParsesCorrectly()
    {
        var doc = "DECL E6POS p1 = {X -100.5, Y -200.3, Z -300.7, A 0, B 0, C 0}";
        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.Single(cache);
        Assert.Equal(-100.5, cache["p1"].x, 0.01);
        Assert.Equal(-200.3, cache["p1"].y, 0.01);
        Assert.Equal(-300.7, cache["p1"].z, 0.01);
    }

    [Fact]
    public void BuildCoordinateCache_EmptyDocument_ReturnsEmpty()
    {
        var cache = RegexHelper.BuildCoordinateCache("");
        Assert.Empty(cache);
    }

    [Fact]
    public void BuildCoordinateCache_NoE6Pos_ReturnsEmpty()
    {
        var doc = @"; Just comments
DEF MyProgram()
    LIN p1
END";
        var cache = RegexHelper.BuildCoordinateCache(doc);
        Assert.Empty(cache);
    }

    // ========================================================================
    // ParsePosition (KRL aggregate)
    // ========================================================================

    [Theory]
    [InlineData("{X 100, Y 200, Z 300, A 0, B 0, C 0}", 100, 200, 300)]
    [InlineData("{X 100.50, Y 200.75, Z 300.25, A 0, B 0, C 0}", 100.5, 200.75, 300.25)]
    [InlineData("{X -50, Y 0, Z 1000, A 0, B 0, C 0}", -50, 0, 1000)]
    public void ParsePosition_ValidValues_ReturnsCorrectCoordinates(string value, double expX, double expY, double expZ)
    {
        var result = RegexHelper.ParsePosition(value);
        Assert.NotNull(result);
        Assert.Equal(expX, result.Value.x, 0.01);
        Assert.Equal(expY, result.Value.y, 0.01);
        Assert.Equal(expZ, result.Value.z, 0.01);
    }

    [Theory]
    [InlineData("")]
    [InlineData("not an e6pos")]
    [InlineData("100,200,300")]   // Missing field names
    public void ParsePosition_InvalidValues_ReturnsNull(string value)
    {
        var result = RegexHelper.ParsePosition(value);
        Assert.Null(result);
    }

    // ========================================================================
    // ArcStart / ArcEnd Regex
    // ========================================================================

    [Theory]
    [InlineData("ArcStart(Job_ID:=1)", true, "1")]
    [InlineData("ArcStart(Job_ID:=5)", true, "5")]
    [InlineData("ArcStart(Job_ID := 3)", true, "3")]              // Extra spaces
    [InlineData("ArcStart", true, null)]                           // No Job_ID
    [InlineData("ArcStart()", true, null)]                         // Empty parens
    [InlineData("arcstart(Job_ID:=1)", true, "1")]                // Lowercase
    [InlineData("LIN p1", false, null)]                            // Not ArcStart
    public void ArcStartRegex_VariousInputs_MatchesCorrectly(string line, bool shouldMatch, string? expectedJobId)
    {
        var match = RegexHelper.ArcStartRegex().Match(line);
        Assert.Equal(shouldMatch, match.Success);
        if (shouldMatch && expectedJobId != null)
        {
            Assert.Equal(expectedJobId, match.Groups[1].Value);
        }
    }

    [Theory]
    [InlineData("ArcEnd", true)]
    [InlineData("arcend", true)]
    [InlineData("  ArcEnd  ", true)]
    [InlineData("ArcStart", false)]
    public void ArcEndRegex_VariousInputs_MatchesCorrectly(string line, bool shouldMatch)
    {
        Assert.Equal(shouldMatch, RegexHelper.ArcEndRegex().IsMatch(line));
    }

    // ========================================================================
    // RobTarget Model Tests (KRL E6POS)
    // ========================================================================

    [Fact]
    public void RobTarget_ToKrlString_ProducesValidFormat()
    {
        var target = new RobTarget
        {
            Name = "p1",
            X = 600.0, Y = 100.0, Z = 500.0,
            A = 0.0, B = 90.0, C = 0.0
        };

        var krl = target.ToKrlString();

        Assert.StartsWith("{X 600.00", krl);
        Assert.Contains("Y 100.00", krl);
        Assert.Contains("Z 500.00", krl);
        Assert.Contains("A 0.00", krl);
        Assert.Contains("B 90.00", krl);
        Assert.Contains("C 0.00", krl);
        Assert.EndsWith("}", krl);

        // Verify it can be parsed back
        var pos = RegexHelper.ParsePosition(krl);
        Assert.NotNull(pos);
        Assert.Equal(600.0, pos.Value.x, 0.01);
        Assert.Equal(100.0, pos.Value.y, 0.01);
        Assert.Equal(500.0, pos.Value.z, 0.01);
    }

    [Fact]
    public void RobTarget_FromPose_StoresEulerAngles()
    {
        var target = RobTarget.FromPose("test", new double[] { 100, 200, 300, 45, 90, 0 });

        Assert.Equal("test", target.Name);
        Assert.Equal(100.0, target.X, 0.01);
        Assert.Equal(200.0, target.Y, 0.01);
        Assert.Equal(300.0, target.Z, 0.01);
        Assert.Equal(45.0, target.A, 0.01);
        Assert.Equal(90.0, target.B, 0.01);
        Assert.Equal(0.0, target.C, 0.01);
    }

    [Fact]
    public void RobTarget_RoundTrip_ToKrlString_ThenParse()
    {
        var original = RobTarget.FromPose("pTest", new double[] { 123.45, -67.89, 456.78, 0, 0, 0 });
        var krl = original.ToKrlString();

        // Put into a document
        var doc = $"DECL E6POS pTest = {krl}";
        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.True(cache.ContainsKey("pTest"));
        Assert.Equal(123.45, cache["pTest"].x, 0.01);
        Assert.Equal(-67.89, cache["pTest"].y, 0.01);
        Assert.Equal(456.78, cache["pTest"].z, 0.01);
    }

    // ========================================================================
    // KrlMotionRegex - CIRC syntax (KRL: CIRC auxPoint, target <, CA angle> <approx>)
    // ========================================================================

    [Theory]
    [InlineData("CIRC pAux, pTarget", true, "pAux", "pTarget", null, null)]
    [InlineData("CIRC pAux, pTarget C_DIS", true, "pAux", "pTarget", null, "C_DIS")]
    [InlineData("CIRC pAux, pTarget, CA 120.0 C_VEL", true, "pAux", "pTarget", "120.0", "C_VEL")]
    [InlineData("CIRC pAux, pTarget, CA 360", true, "pAux", "pTarget", "360", null)]
    [InlineData("LIN p1 C_DIS", true, "p1", null, null, "C_DIS")]
    [InlineData("PTP pHome C_PTP", true, "pHome", null, null, "C_PTP")]
    [InlineData("PTP pHome", true, "pHome", null, null, null)]
    public void KrlMotionRegex_VariousInputs_MatchesCorrectly(
        string line, bool shouldMatch, string? group2, string? group3, string? caAngle, string? approx)
    {
        var match = RegexHelper.KrlMotionRegex().Match(line);
        Assert.Equal(shouldMatch, match.Success);
        if (shouldMatch)
        {
            Assert.Equal(group2 ?? "", match.Groups[2].Success ? match.Groups[2].Value : "");
            if (group3 != null)
                Assert.Equal(group3, match.Groups[3].Value);
            else
                Assert.False(match.Groups[3].Success);
            if (caAngle != null)
                Assert.Equal(caAngle, match.Groups[4].Value);
            else
                Assert.False(match.Groups[4].Success);
            if (approx != null)
                Assert.Equal(approx, match.Groups[5].Value);
            else
                Assert.False(match.Groups[5].Success);
        }
    }

    // ========================================================================
    // ExtractApproximation (KRL)
    // ========================================================================

    [Theory]
    [InlineData("PTP p1 C_PTP", ApproximationType.C_PTP)]
    [InlineData("LIN p1 C_DIS", ApproximationType.C_DIS)]
    [InlineData("LIN p1 C_VEL", ApproximationType.C_VEL)]
    [InlineData("LIN p1 C_ORI", ApproximationType.C_ORI)]
    [InlineData("PTP p1", ApproximationType.EXACT)]
    [InlineData("CIRC pAux, pTarget C_DIS", ApproximationType.C_DIS)]
    [InlineData("CIRC pAux, pTarget, CA 180 C_VEL", ApproximationType.C_VEL)]
    [InlineData("CIRC pAux, pTarget", ApproximationType.EXACT)]
    public void ExtractApproximation_VariousInputs_ReturnsCorrectType(string line, ApproximationType expected)
    {
        var result = RegexHelper.ExtractApproximation(line);
        Assert.Equal(expected, result);
    }
}
