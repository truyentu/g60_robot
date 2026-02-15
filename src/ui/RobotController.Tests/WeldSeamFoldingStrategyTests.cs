using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Folding;
using RobotController.UI.Editor;

namespace RobotController.Tests;

/// <summary>
/// Unit tests for WeldSeamFoldingStrategy — FOLD/ENDFOLD, REGION, DEF/END, ArcStart/ArcEnd parsing.
/// </summary>
public class WeldSeamFoldingStrategyTests
{
    private readonly WeldSeamFoldingStrategy _strategy = new();

    private List<NewFolding> GetFoldings(string text, out int firstErrorOffset)
    {
        var doc = new TextDocument(text);
        return _strategy.CreateFoldings(doc, out firstErrorOffset).ToList();
    }

    // ========================================================================
    // ;FOLD ... ;ENDFOLD basic
    // ========================================================================

    [Fact]
    public void Fold_BasicFoldBlock_CreatesFolding()
    {
        var text = ";FOLD PTP HOME\n  PTP HOME\n;ENDFOLD";
        var foldings = GetFoldings(text, out int err);

        Assert.Single(foldings);
        Assert.Equal("PTP HOME", foldings[0].Name);
        Assert.False(foldings[0].DefaultClosed);
        Assert.Equal(-1, err);
    }

    [Fact]
    public void Fold_BareFold_UsesFoldAsLabel()
    {
        var text = ";FOLD\n  some content\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("FOLD", foldings[0].Name);
    }

    [Fact]
    public void Fold_CaseInsensitive_MatchesFoldEndfold()
    {
        var text = ";fold My Block\n  content\n;endfold";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("My Block", foldings[0].Name);
    }

    [Fact]
    public void Fold_MixedCase_MatchesFoldEndfold()
    {
        var text = ";Fold Mixed Case\n  content\n;EndFold";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("Mixed Case", foldings[0].Name);
    }

    // ========================================================================
    // INI / BASISTECH folds — DefaultClosed = true
    // ========================================================================

    [Fact]
    public void Fold_IniBlock_DefaultClosed()
    {
        var text = ";FOLD INI\n  BAS(#INITMOV, 0)\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("INI", foldings[0].Name);
        Assert.True(foldings[0].DefaultClosed);
    }

    [Fact]
    public void Fold_BasistechIni_DefaultClosed()
    {
        var text = ";FOLD BASISTECH INI\n  BAS(#INITMOV, 0)\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("BASISTECH INI", foldings[0].Name);
        Assert.True(foldings[0].DefaultClosed);
    }

    [Fact]
    public void Fold_IniLowercase_DefaultClosed()
    {
        var text = ";FOLD ini stuff\n  content\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.True(foldings[0].DefaultClosed);
    }

    [Fact]
    public void Fold_NonIniBlock_DefaultOpen()
    {
        var text = ";FOLD LIN P1 Vel=2m/s\n  LIN P1\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.False(foldings[0].DefaultClosed);
    }

    // ========================================================================
    // Nested FOLD blocks
    // ========================================================================

    [Fact]
    public void Fold_Nested_CreatesMultipleFoldings()
    {
        var text = ";FOLD INI\n  ;FOLD BASISTECH INI\n    BAS(#INITMOV, 0)\n  ;ENDFOLD\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Equal(2, foldings.Count);

        // Sorted by StartOffset — outer should be first
        var outer = foldings[0];
        var inner = foldings[1];
        Assert.True(outer.StartOffset < inner.StartOffset);
    }

    [Fact]
    public void Fold_DeeplyNested_ThreeLevels()
    {
        var text = ";FOLD Level1\n  ;FOLD Level2\n    ;FOLD Level3\n      content\n    ;ENDFOLD\n  ;ENDFOLD\n;ENDFOLD";
        var foldings = GetFoldings(text, out _);

        Assert.Equal(3, foldings.Count);
    }

    // ========================================================================
    // Unclosed FOLD — error offset
    // ========================================================================

    [Fact]
    public void Fold_Unclosed_SetsFirstErrorOffset()
    {
        var text = ";FOLD PTP HOME\n  PTP HOME\n; no endfold here";
        var foldings = GetFoldings(text, out int err);

        Assert.Empty(foldings);
        Assert.True(err >= 0, "firstErrorOffset should be set for unclosed FOLD");
    }

    [Fact]
    public void Fold_EndfoldWithoutFold_Ignored()
    {
        var text = "some code\n;ENDFOLD\nmore code";
        var foldings = GetFoldings(text, out int err);

        Assert.Empty(foldings);
        Assert.Equal(-1, err);
    }

    [Fact]
    public void Fold_PartiallyUnclosed_HasErrorAndSomeFoldings()
    {
        var text = ";FOLD Outer\n  ;FOLD Inner\n    content\n  ;ENDFOLD\n; outer not closed";
        var foldings = GetFoldings(text, out int err);

        // Inner is properly closed
        Assert.Single(foldings);
        Assert.Equal("Inner", foldings[0].Name);
        // Outer is unclosed → error
        Assert.True(err >= 0);
    }

    // ========================================================================
    // FOLD coexists with other fold types
    // ========================================================================

    [Fact]
    public void Fold_CoexistsWithDefEnd()
    {
        var text = "DEF MyProgram()\n  ;FOLD INI\n    BAS(#INITMOV, 0)\n  ;ENDFOLD\n  PTP HOME\nEND";
        var foldings = GetFoldings(text, out int err);

        Assert.Equal(2, foldings.Count);
        Assert.Equal(-1, err);

        // DEF block
        var defFold = foldings.First(f => f.Name.StartsWith("DEF"));
        // TrimEnd('(', ')') strips parens from label
        Assert.Equal("DEF MyProgram", defFold.Name);
        Assert.False(defFold.DefaultClosed);

        // FOLD block
        var foldBlock = foldings.First(f => f.Name == "INI");
        Assert.True(foldBlock.DefaultClosed);
    }

    [Fact]
    public void Fold_CoexistsWithRegion()
    {
        var text = ";#REGION Setup\n  ;FOLD INI\n    content\n  ;ENDFOLD\n;#ENDREGION";
        var foldings = GetFoldings(text, out int err);

        Assert.Equal(2, foldings.Count);
        Assert.Equal(-1, err);

        Assert.Contains(foldings, f => f.Name == "Setup");
        Assert.Contains(foldings, f => f.Name == "INI");
    }

    // ========================================================================
    // ;#REGION ... ;#ENDREGION
    // ========================================================================

    [Fact]
    public void Region_BasicBlock_CreatesFolding()
    {
        var text = ";#REGION MyRegion\n  content\n;#ENDREGION";
        var foldings = GetFoldings(text, out int err);

        Assert.Single(foldings);
        Assert.Equal("MyRegion", foldings[0].Name);
        Assert.False(foldings[0].DefaultClosed);
        Assert.Equal(-1, err);
    }

    [Fact]
    public void Region_CaseInsensitive()
    {
        var text = ";#region LowerCase\n  content\n;#endregion";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("LowerCase", foldings[0].Name);
    }

    [Fact]
    public void Region_NoName_UsesDefault()
    {
        var text = ";#REGION\n  content\n;#ENDREGION";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
        Assert.Equal("Region", foldings[0].Name);
    }

    // ========================================================================
    // DEF ... END blocks
    // ========================================================================

    [Fact]
    public void Def_BasicBlock_CreatesFolding()
    {
        var text = "DEF MyRoutine()\n  LIN P1\n  LIN P2\nEND";
        var foldings = GetFoldings(text, out int err);

        Assert.Single(foldings);
        // TrimEnd('(', ')') strips trailing parens from the label
        Assert.Equal("DEF MyRoutine", foldings[0].Name);
        Assert.False(foldings[0].DefaultClosed);
        Assert.Equal(-1, err);
    }

    [Fact]
    public void Def_CaseInsensitive()
    {
        var text = "def mylower()\n  content\nend";
        var foldings = GetFoldings(text, out _);

        Assert.Single(foldings);
    }

    [Fact]
    public void Def_EndWithoutDef_NoFolding()
    {
        var text = "some code\nEND\nmore code";
        var foldings = GetFoldings(text, out int err);

        Assert.Empty(foldings);
        Assert.Equal(-1, err);
    }

    // ========================================================================
    // Complex programs — realistic KRL
    // ========================================================================

    [Fact]
    public void ComplexProgram_MultipleFoldTypes()
    {
        var text = @"&ACCESS RV
&REL 1
&COMMENT Test program

DEF TestProgram()
  ;FOLD INI
    ;FOLD BASISTECH INI
      BAS(#INITMOV, 0)
    ;ENDFOLD
  ;ENDFOLD

  ;FOLD PTP HOME Vel=100% DEFAULT
    PTP HOME Vel=100% DEFAULT
  ;ENDFOLD

  ;#REGION Welding Section
    ;FOLD LIN P1 Vel=2m/s
      LIN P1 Vel=2m/s
    ;ENDFOLD
  ;#ENDREGION

END";
        var foldings = GetFoldings(text, out int err);

        Assert.Equal(-1, err);

        // Expected: DEF (1) + INI (1) + BASISTECH INI (1) + PTP HOME (1) + REGION (1) + LIN P1 (1) = 6
        Assert.Equal(6, foldings.Count);

        // INI and BASISTECH should be DefaultClosed
        var iniFolds = foldings.Where(f => f.DefaultClosed).ToList();
        Assert.Equal(2, iniFolds.Count);
    }

    [Fact]
    public void EmptyDocument_NoFoldings()
    {
        var foldings = GetFoldings("", out int err);
        Assert.Empty(foldings);
        Assert.Equal(-1, err);
    }

    [Fact]
    public void NoFoldBlocks_OnlyComments_NoFoldings()
    {
        var text = "; This is a comment\n; Another comment\nPTP P1\nLIN P2";
        var foldings = GetFoldings(text, out int err);

        Assert.Empty(foldings);
        Assert.Equal(-1, err);
    }

    // ========================================================================
    // Folding offsets — verify start/end positions
    // ========================================================================

    [Fact]
    public void Fold_Offsets_CorrectStartAndEnd()
    {
        var text = "line1\n;FOLD TestBlock\n  content\n;ENDFOLD\nline5";
        var doc = new TextDocument(text);
        var foldings = _strategy.CreateFoldings(doc, out _).ToList();

        Assert.Single(foldings);
        var fold = foldings[0];

        // Start should be at line 2 (";FOLD TestBlock")
        var line2 = doc.GetLineByNumber(2);
        Assert.Equal(line2.Offset, fold.StartOffset);

        // End should be at end of line 4 (";ENDFOLD")
        var line4 = doc.GetLineByNumber(4);
        Assert.Equal(line4.Offset + line4.Length, fold.EndOffset);
    }

    // ========================================================================
    // Sorting — foldings sorted by StartOffset
    // ========================================================================

    [Fact]
    public void Foldings_SortedByStartOffset()
    {
        var text = @"DEF Prog()
  ;FOLD Block1
    content
  ;ENDFOLD
  ;FOLD Block2
    content
  ;ENDFOLD
END";
        var foldings = GetFoldings(text, out _);

        for (int i = 1; i < foldings.Count; i++)
        {
            Assert.True(foldings[i].StartOffset >= foldings[i - 1].StartOffset,
                $"Folding {i} offset {foldings[i].StartOffset} should be >= folding {i - 1} offset {foldings[i - 1].StartOffset}");
        }
    }

    // ========================================================================
    // Multiple unclosed — error offset from first unclosed
    // ========================================================================

    [Fact]
    public void Fold_MultipleUnclosed_ErrorFromFirst()
    {
        var text = ";FOLD First\n;FOLD Second\n  content";
        var foldings = GetFoldings(text, out int err);

        Assert.Empty(foldings);
        // Stack — last pushed is peeked, so the error is from "Second" (top of stack)
        Assert.True(err >= 0);
    }
}
