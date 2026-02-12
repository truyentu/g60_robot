using RobotController.UI.Helpers;
using RobotController.UI.Models;

namespace RobotController.Tests;

/// <summary>
/// Unit tests for RegexHelper - verifying robustness with messy inputs.
/// Covers: tabs, extra spaces, trailing comments, mixed case, malformed syntax.
/// </summary>
public class RegexHelperTests
{
    // ========================================================================
    // ExtractTargetName - Basic Cases
    // ========================================================================

    [Theory]
    [InlineData("MoveL p1, v100, fine, tool0;", "p1")]
    [InlineData("MoveJ pHome, vmax, z100, tool0;", "pHome")]
    [InlineData("MoveC pArc, v50, z10, tool0;", "pArc")]
    public void ExtractTargetName_StandardInput_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // ExtractTargetName - Messy Inputs (Leader's requirement)
    // ========================================================================

    [Theory]
    [InlineData("    MoveL p1, v100, fine, tool0;", "p1")]                    // Leading spaces
    [InlineData("\tMoveL p1, v100, fine, tool0;", "p1")]                      // Leading tab
    [InlineData("\t\t  MoveL p1, v100, fine, tool0;", "p1")]                  // Tabs + spaces
    [InlineData("MoveL   p1 , v100 , fine , tool0 ;", "p1")]                  // Extra spaces everywhere
    [InlineData("MoveL\tp1,\tv100,\tfine,\ttool0;", "p1")]                    // Tabs as separators
    [InlineData("MoveL  \t p1  ,v100, fine, tool0;", "p1")]                   // Mixed tabs+spaces
    public void ExtractTargetName_MessyWhitespace_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("MoveL P1, v100, fine, tool0;", "P1")]                        // Uppercase target
    [InlineData("movel p1, v100, fine, tool0;", "p1")]                        // Lowercase motion
    [InlineData("MOVEL P1, V100, FINE, TOOL0;", "P1")]                        // All uppercase
    [InlineData("moveL p1, v100, fine, tool0;", "p1")]                        // Mixed case motion
    [InlineData("MoveL my_target_1, v100, fine, tool0;", "my_target_1")]      // Underscore in name
    public void ExtractTargetName_CaseVariations_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("MoveL p1, v100, fine, tool0; ! go to start", "p1")]          // ABB comment after
    [InlineData("MoveL p1, v100, fine, tool0; ; KUKA comment", "p1")]         // KUKA comment after
    [InlineData("MoveL p1, v100, fine, tool0; //C-style comment", "p1")]      // C-style comment after
    [InlineData("  MoveL p1, v100, fine, tool0 ;  ! approach point  ", "p1")] // Whitespace + comment
    public void ExtractTargetName_TrailingComments_ReturnsTargetName(string line, string expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    [Theory]
    [InlineData("! This is a comment", null)]                                  // Comment-only line
    [InlineData("; KUKA comment", null)]                                       // KUKA comment
    [InlineData("", null)]                                                     // Empty line
    [InlineData("   ", null)]                                                  // Whitespace-only
    [InlineData("ArcStart(Job_ID:=1);", null)]                                 // Process command
    [InlineData("ArcEnd;", null)]                                              // Process command
    [InlineData("DEF WeldProgram()", null)]                                    // Routine definition
    [InlineData("END", null)]                                                  // End block
    [InlineData("CONST robtarget p1 := [[100,200,300],...];", null)]           // Variable definition (not motion)
    [InlineData("IF flag = TRUE THEN", null)]                                  // Control flow
    public void ExtractTargetName_NonMotionLines_ReturnsNull(string line, string? expected)
    {
        var result = RegexHelper.ExtractTargetName(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // ExtractMotionType
    // ========================================================================

    [Theory]
    [InlineData("MoveL p1, v100, fine, tool0;", MotionType.MoveL)]
    [InlineData("MoveJ pHome, vmax, z100, tool0;", MotionType.MoveJ)]
    [InlineData("MoveC pArc, v50, z10, tool0;", MotionType.MoveC)]
    [InlineData("\t  movel  p1 , v100, fine, tool0;", MotionType.MoveL)]      // Messy + lowercase
    [InlineData("! comment line", null)]
    [InlineData("ArcStart;", null)]
    public void ExtractMotionType_VariousInputs_ReturnsCorrectType(string line, MotionType? expected)
    {
        var result = RegexHelper.ExtractMotionType(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // IsMotionInstruction
    // ========================================================================

    [Theory]
    [InlineData("MoveL p1, v100, fine, tool0;", true)]
    [InlineData("\tMoveJ pHome, vmax, z100, tool0; ! comment", true)]
    [InlineData("! just a comment", false)]
    [InlineData("ArcStart(Job_ID:=1);", false)]
    [InlineData("CONST robtarget p1 := [[100,0,0],...];", false)]
    [InlineData("", false)]
    public void IsMotionInstruction_VariousInputs_ReturnsCorrectResult(string line, bool expected)
    {
        var result = RegexHelper.IsMotionInstruction(line);
        Assert.Equal(expected, result);
    }

    // ========================================================================
    // FindVariableDefinition - Core Touch-Up logic
    // ========================================================================

    [Fact]
    public void FindVariableDefinition_StandardDefinition_FindsValueRange()
    {
        var doc = "CONST robtarget p1 := [[600.00,100.00,500.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "p1");

        Assert.NotNull(result);
        var (offset, length, value) = result.Value;
        Assert.StartsWith("[[600.00", value);
        Assert.EndsWith("]]", value);
        // Verify the extracted range is correct by checking the document
        Assert.Equal(value, doc.Substring(offset, length));
    }

    [Fact]
    public void FindVariableDefinition_WithIndentation_PreservesContext()
    {
        var doc = @"! Program header
    CONST robtarget p1 := [[100.00,200.00,300.00],[1.0,0.0,0.0,0.0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget p2 := [[400.00,500.00,600.00],[1.0,0.0,0.0,0.0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";

        var result = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(result);
        var (offset, length, value) = result.Value;

        // Verify only the value range is returned, not indentation
        Assert.StartsWith("[[400.00", value);
        Assert.DoesNotContain("CONST", value);
        Assert.DoesNotContain("robtarget", value);
    }

    [Fact]
    public void FindVariableDefinition_VarQualifier_Works()
    {
        var doc = "VAR robtarget pMid := [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "pMid");
        Assert.NotNull(result);
        Assert.StartsWith("[[0,0,0]", result.Value.currentValue);
    }

    [Fact]
    public void FindVariableDefinition_PersQualifier_Works()
    {
        var doc = "PERS robtarget pSaved := [[10,20,30],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "pSaved");
        Assert.NotNull(result);
    }

    [Fact]
    public void FindVariableDefinition_NoQualifier_Works()
    {
        var doc = "robtarget pTemp := [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "pTemp");
        Assert.NotNull(result);
    }

    [Fact]
    public void FindVariableDefinition_NotFound_ReturnsNull()
    {
        var doc = "CONST robtarget p1 := [[100,200,300],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "pNotExist");
        Assert.Null(result);
    }

    [Fact]
    public void FindVariableDefinition_CaseInsensitive_MatchesTarget()
    {
        var doc = "CONST robtarget pHome := [[500,0,800],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var result = RegexHelper.FindVariableDefinition(doc, "PHOME");
        Assert.NotNull(result);
    }

    // ========================================================================
    // FindVariableDefinition - Formatting Preservation (Action Item #2)
    // ========================================================================

    [Fact]
    public void TouchUp_Simulation_PreservesDocumentFormatting()
    {
        // Simulate the exact Touch-Up workflow
        var originalDoc = @"! Robot Program - Welding Example
! Variable Definitions
    CONST robtarget pHome := [[500.00,0.00,800.00],[1.0000,0.0000,0.0000,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    CONST robtarget p1 := [[600.00,100.00,500.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

DEF WeldProgram()
    MoveJ pHome, vmax, z100, tool0;
    MoveL p1, v1000, z50, tool0;
END";

        // Step 1: Find p1's value range
        var definition = RegexHelper.FindVariableDefinition(originalDoc, "p1");
        Assert.NotNull(definition);
        var (offset, length, oldValue) = definition.Value;

        // Step 2: Simulate new value from Touch-Up
        var newValue = "[[650.00,150.00,480.00],[0.7071,0.0000,0.7071,0.0000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]";

        // Step 3: Replace (same as ProgramEditorViewModel.ModifyPositionAsync)
        var newDoc = originalDoc.Remove(offset, length).Insert(offset, newValue);

        // VERIFY: Indentation preserved
        Assert.Contains("    CONST robtarget p1 := [[650.00", newDoc);

        // VERIFY: Other lines untouched
        Assert.Contains("    CONST robtarget pHome := [[500.00", newDoc);
        Assert.Contains("DEF WeldProgram()", newDoc);
        Assert.Contains("    MoveJ pHome, vmax, z100, tool0;", newDoc);
        Assert.Contains("    MoveL p1, v1000, z50, tool0;", newDoc);
        Assert.Contains("END", newDoc);

        // VERIFY: Comment preserved
        Assert.Contains("! Robot Program - Welding Example", newDoc);

        // VERIFY: pHome definition NOT changed
        var pHomeResult = RegexHelper.FindVariableDefinition(newDoc, "pHome");
        Assert.NotNull(pHomeResult);
        Assert.Contains("500.00", pHomeResult.Value.currentValue);
    }

    [Fact]
    public void TouchUp_Simulation_MultipleUpdates_PreservesStructure()
    {
        var doc = @"CONST robtarget p1 := [[100,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p2 := [[200,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p3 := [[300,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";

        // Update p2
        var def = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(def);
        var newVal = "[[250,50,10],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]";
        doc = doc.Remove(def.Value.offset, def.Value.length).Insert(def.Value.offset, newVal);

        // Verify p1 and p3 untouched
        var p1 = RegexHelper.FindVariableDefinition(doc, "p1");
        var p3 = RegexHelper.FindVariableDefinition(doc, "p3");
        Assert.NotNull(p1);
        Assert.NotNull(p3);
        Assert.Contains("100,0,0", p1.Value.currentValue);
        Assert.Contains("300,0,0", p3.Value.currentValue);

        // Verify p2 updated
        var p2 = RegexHelper.FindVariableDefinition(doc, "p2");
        Assert.NotNull(p2);
        Assert.Contains("250,50,10", p2.Value.currentValue);
    }

    // ========================================================================
    // BuildCoordinateCache
    // ========================================================================

    [Fact]
    public void BuildCoordinateCache_MultipleTargets_ReturnsAll()
    {
        var doc = @"CONST robtarget p1 := [[100.0,200.0,300.0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget p2 := [[400.0,500.0,600.0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
VAR robtarget p3 := [[700.0,800.0,900.0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";

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
        var doc = "CONST robtarget p1 := [[1.5E2,2.0E1,3.0E-1],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.Single(cache);
        Assert.Equal(150.0, cache["p1"].x, 0.01);
        Assert.Equal(20.0, cache["p1"].y, 0.01);
        Assert.Equal(0.3, cache["p1"].z, 0.01);
    }

    [Fact]
    public void BuildCoordinateCache_NegativeCoordinates_ParsesCorrectly()
    {
        var doc = "CONST robtarget p1 := [[-100.5,-200.3,-300.7],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];";
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
    public void BuildCoordinateCache_NoRobTargets_ReturnsEmpty()
    {
        var doc = @"! Just comments
DEF MyProgram()
    MoveL p1, v100, fine, tool0;
END";
        var cache = RegexHelper.BuildCoordinateCache(doc);
        Assert.Empty(cache);
    }

    // ========================================================================
    // ParsePosition
    // ========================================================================

    [Theory]
    [InlineData("[[100,200,300],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]", 100, 200, 300)]
    [InlineData("[[100.50,200.75,300.25],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]", 100.5, 200.75, 300.25)]
    [InlineData("[[-50,0,1000],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]", -50, 0, 1000)]
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
    [InlineData("not a robtarget")]
    [InlineData("[100,200,300]")]  // Missing outer bracket
    public void ParsePosition_InvalidValues_ReturnsNull(string value)
    {
        var result = RegexHelper.ParsePosition(value);
        Assert.Null(result);
    }

    // ========================================================================
    // ArcStart / ArcEnd Regex
    // ========================================================================

    [Theory]
    [InlineData("ArcStart(Job_ID:=1);", true, "1")]
    [InlineData("ArcStart(Job_ID:=5);", true, "5")]
    [InlineData("ArcStart(Job_ID := 3);", true, "3")]        // Extra spaces
    [InlineData("ArcStart;", true, null)]                      // No Job_ID
    [InlineData("ArcStart()", true, null)]                     // Empty parens
    [InlineData("arcstart(Job_ID:=1);", true, "1")]           // Lowercase
    [InlineData("MoveL p1;", false, null)]                     // Not ArcStart
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
    [InlineData("ArcEnd;", true)]
    [InlineData("ArcEnd", true)]
    [InlineData("arcend;", true)]
    [InlineData("  ArcEnd  ;  ", true)]
    [InlineData("ArcStart;", false)]
    public void ArcEndRegex_VariousInputs_MatchesCorrectly(string line, bool shouldMatch)
    {
        Assert.Equal(shouldMatch, RegexHelper.ArcEndRegex().IsMatch(line));
    }

    // ========================================================================
    // RobTarget Model Tests
    // ========================================================================

    [Fact]
    public void RobTarget_ToRplString_ProducesValidFormat()
    {
        var target = new RobTarget
        {
            Name = "p1",
            X = 600.0, Y = 100.0, Z = 500.0,
            Q1 = 0.7071, Q2 = 0.0, Q3 = 0.7071, Q4 = 0.0,
            Cf1 = 0, Cf4 = 0, Cf6 = 0, Cfx = 0
        };

        var rpl = target.ToRplString();

        Assert.StartsWith("[[600.00,100.00,500.00]", rpl);
        Assert.Contains("[0.7071,0.0000,0.7071,0.0000]", rpl);
        Assert.Contains("[0,0,0,0]", rpl);
        Assert.EndsWith("]]", rpl);

        // Verify it can be parsed back
        var pos = RegexHelper.ParsePosition(rpl);
        Assert.NotNull(pos);
        Assert.Equal(600.0, pos.Value.x, 0.01);
        Assert.Equal(100.0, pos.Value.y, 0.01);
        Assert.Equal(500.0, pos.Value.z, 0.01);
    }

    [Fact]
    public void RobTarget_FromPose_ConvertsEulerToQuaternion()
    {
        // Identity orientation (0,0,0 Euler = [1,0,0,0] quaternion)
        var target = RobTarget.FromPose("test", new double[] { 100, 200, 300, 0, 0, 0 });

        Assert.Equal("test", target.Name);
        Assert.Equal(100.0, target.X, 0.01);
        Assert.Equal(200.0, target.Y, 0.01);
        Assert.Equal(300.0, target.Z, 0.01);
        Assert.Equal(1.0, target.Q1, 0.01);
        Assert.Equal(0.0, target.Q2, 0.01);
        Assert.Equal(0.0, target.Q3, 0.01);
        Assert.Equal(0.0, target.Q4, 0.01);
    }

    [Fact]
    public void RobTarget_RoundTrip_ToRplString_ThenParse()
    {
        var original = RobTarget.FromPose("pTest", new double[] { 123.45, -67.89, 456.78, 0, 0, 0 });
        var rpl = original.ToRplString();

        // Put into a document
        var doc = $"CONST robtarget pTest := {rpl};";
        var cache = RegexHelper.BuildCoordinateCache(doc);

        Assert.True(cache.ContainsKey("pTest"));
        Assert.Equal(123.45, cache["pTest"].x, 0.01);
        Assert.Equal(-67.89, cache["pTest"].y, 0.01);
        Assert.Equal(456.78, cache["pTest"].z, 0.01);
    }
}
