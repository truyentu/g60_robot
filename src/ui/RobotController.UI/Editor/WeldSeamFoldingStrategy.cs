using ICSharpCode.AvalonEdit.Document;
using ICSharpCode.AvalonEdit.Folding;
using RobotController.UI.Helpers;

namespace RobotController.UI.Editor;

/// <summary>
/// Semantic folding strategy that groups ArcStart...ArcEnd blocks
/// and calculates weld seam length from coordinate definitions.
/// Also folds DEF...END routine blocks.
/// Folding label: "Weld Seam (Job: 1) - Length: 145.2 mm"
/// </summary>
public class WeldSeamFoldingStrategy
{
    /// <summary>
    /// Update foldings on the given FoldingManager using the document content.
    /// </summary>
    public void UpdateFoldings(FoldingManager manager, TextDocument document)
    {
        var foldings = CreateFoldings(document, out int firstErrorOffset);
        manager.UpdateFoldings(foldings, firstErrorOffset);
    }

    /// <summary>
    /// Create folding regions by scanning for semantic blocks.
    /// </summary>
    public IEnumerable<NewFolding> CreateFoldings(TextDocument document, out int firstErrorOffset)
    {
        firstErrorOffset = -1;
        var newFoldings = new List<NewFolding>();
        var seamStack = new Stack<FoldingStartInfo>();
        var routineStack = new Stack<FoldingStartInfo>();
        var regionStack = new Stack<FoldingStartInfo>();

        // Pre-parse all E6POS coordinates into a lookup
        var coordinateCache = RegexHelper.BuildCoordinateCache(document.Text);

        for (int i = 1; i <= document.LineCount; i++)
        {
            var line = document.GetLineByNumber(i);
            string text = document.GetText(line.Offset, line.Length).Trim();

            // Check for ;#REGION ... ;#ENDREGION (KRL comment style)
            if (text.StartsWith(";#REGION", StringComparison.OrdinalIgnoreCase))
            {
                var regionName = text.Length > 8 ? text[8..].Trim() : "Region";
                regionStack.Push(new FoldingStartInfo
                {
                    Offset = line.Offset,
                    LineNumber = i,
                    Label = regionName
                });
                continue;
            }

            if (text.StartsWith(";#ENDREGION", StringComparison.OrdinalIgnoreCase) &&
                regionStack.Count > 0)
            {
                var startInfo = regionStack.Pop();
                int endOffset = line.Offset + line.Length;
                var folding = new NewFolding(startInfo.Offset, endOffset)
                {
                    Name = startInfo.Label,
                    DefaultClosed = false
                };
                newFoldings.Add(folding);
                continue;
            }

            // Check for ArcStart
            var startMatch = RegexHelper.ArcStartRegex().Match(text);
            if (startMatch.Success)
            {
                string jobId = startMatch.Groups[1].Success ? startMatch.Groups[1].Value : "?";
                seamStack.Push(new FoldingStartInfo
                {
                    Offset = line.Offset,
                    LineNumber = i,
                    Label = jobId
                });
                continue;
            }

            // Check for ArcEnd
            if (RegexHelper.ArcEndRegex().IsMatch(text) && seamStack.Count > 0)
            {
                var startInfo = seamStack.Pop();
                int endOffset = line.Offset + line.Length;

                // Calculate weld seam length
                double length = CalculateSeamLength(document, startInfo.LineNumber, i, coordinateCache);

                var folding = new NewFolding(startInfo.Offset, endOffset)
                {
                    Name = $"Weld Seam (Job: {startInfo.Label}) - Length: {length:F1} mm",
                    DefaultClosed = false
                };
                newFoldings.Add(folding);
                continue;
            }

            // Check for DEF...END blocks
            if (text.StartsWith("DEF ", StringComparison.OrdinalIgnoreCase))
            {
                var routineName = text.Length > 4 ? text[4..].TrimEnd('(', ')') : "Routine";
                routineStack.Push(new FoldingStartInfo
                {
                    Offset = line.Offset,
                    LineNumber = i,
                    Label = routineName
                });
            }
            else if (text.Equals("END", StringComparison.OrdinalIgnoreCase) && routineStack.Count > 0)
            {
                var startInfo = routineStack.Pop();
                int endOffset = line.Offset + line.Length;
                var folding = new NewFolding(startInfo.Offset, endOffset)
                {
                    Name = $"DEF {startInfo.Label}",
                    DefaultClosed = false
                };
                newFoldings.Add(folding);
            }
        }

        // Mark unclosed blocks as errors
        if (seamStack.Count > 0)
        {
            firstErrorOffset = seamStack.Peek().Offset;
        }

        newFoldings.Sort((a, b) => a.StartOffset.CompareTo(b.StartOffset));
        return newFoldings;
    }

    private static double CalculateSeamLength(
        TextDocument doc,
        int startLine,
        int endLine,
        Dictionary<string, (double x, double y, double z)> coords)
    {
        double totalLength = 0;
        (double x, double y, double z)? lastPoint = null;

        for (int i = startLine + 1; i < endLine; i++)
        {
            var line = doc.GetLineByNumber(i);
            string text = doc.GetText(line.Offset, line.Length);

            var targetName = RegexHelper.ExtractTargetName(text);
            if (targetName == null) continue;

            // Only count LIN (linear) for seam length
            var motionType = RegexHelper.ExtractMotionType(text);
            if (motionType != Models.MotionType.LIN) continue;

            if (coords.TryGetValue(targetName, out var currentPoint))
            {
                if (lastPoint.HasValue)
                {
                    double dx = currentPoint.x - lastPoint.Value.x;
                    double dy = currentPoint.y - lastPoint.Value.y;
                    double dz = currentPoint.z - lastPoint.Value.z;
                    totalLength += Math.Sqrt(dx * dx + dy * dy + dz * dz);
                }
                lastPoint = currentPoint;
            }
        }

        return totalLength;
    }

    private struct FoldingStartInfo
    {
        public int Offset;
        public int LineNumber;
        public string Label;
    }
}
