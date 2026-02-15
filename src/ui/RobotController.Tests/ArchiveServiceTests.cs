using System.IO;
using System.IO.Compression;
using RobotController.UI.Services;

namespace RobotController.Tests;

/// <summary>
/// Unit tests for ArchiveService — ZIP archive/restore/compare operations.
/// Uses temp directories for isolation.
/// </summary>
public class ArchiveServiceTests : IDisposable
{
    private readonly string _tempDir;
    private readonly string _workspaceRoot;
    private readonly WorkspaceService _workspace;
    private readonly ArchiveService _archive;

    public ArchiveServiceTests()
    {
        _tempDir = Path.Combine(Path.GetTempPath(), "RobotCtrl_Test_" + Guid.NewGuid().ToString("N")[..8]);
        _workspaceRoot = Path.Combine(_tempDir, "workspace");
        Directory.CreateDirectory(_workspaceRoot);

        _workspace = new WorkspaceService(_workspaceRoot);
        _archive = new ArchiveService(_workspace);
    }

    public void Dispose()
    {
        try
        {
            if (Directory.Exists(_tempDir))
                Directory.Delete(_tempDir, recursive: true);
        }
        catch { }
    }

    private string CreateTestFile(string relativePath, string content = "test content")
    {
        var fullPath = Path.Combine(_workspaceRoot, relativePath);
        Directory.CreateDirectory(Path.GetDirectoryName(fullPath)!);
        File.WriteAllText(fullPath, content);
        return fullPath;
    }

    private string GetZipPath(string name = "test_archive") =>
        Path.Combine(_tempDir, name + ".zip");

    // ========================================================================
    // ArchiveAll
    // ========================================================================

    [Fact]
    public void ArchiveAll_CreatesZipWithAllFiles()
    {
        CreateTestFile("R1/Program/WeldSeam01.src", "DEF WeldSeam01()\nEND");
        CreateTestFile("R1/Program/WeldSeam01.dat", "DEFDAT WeldSeam01\nENDDAT");
        CreateTestFile("Config/system.yaml", "version: 1");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        Assert.True(File.Exists(zipPath));

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Equal(3, entries.Count);
    }

    [Fact]
    public void ArchiveAll_ExcludesLogDirectory()
    {
        CreateTestFile("R1/Program/test.src", "content");
        CreateTestFile("Log/system.log", "log entry");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.DoesNotContain(entries, e => e.RelativePath.StartsWith("Log/", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void ArchiveAll_ExcludesCatalogDirectory()
    {
        CreateTestFile("R1/Program/test.src", "content");
        CreateTestFile("Catalog/kuka_kr6/robot.yaml", "robot data");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.DoesNotContain(entries, e => e.RelativePath.StartsWith("Catalog/", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void ArchiveAll_IncludesManifest()
    {
        CreateTestFile("R1/Program/test.src", "content");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Manifest is excluded from ListArchiveContents but exists in ZIP
        using var zip = System.IO.Compression.ZipFile.OpenRead(zipPath);
        Assert.Contains(zip.Entries, e => e.FullName == "_archive_manifest.json");
    }

    [Fact]
    public void ArchiveAll_OverwritesExistingZip()
    {
        CreateTestFile("R1/Program/test.src", "content");
        var zipPath = GetZipPath();

        // Create first archive
        _archive.ArchiveAll(zipPath);
        var firstSize = new FileInfo(zipPath).Length;

        // Add more files and archive again
        CreateTestFile("R1/Program/test2.src", "more content");
        _archive.ArchiveAll(zipPath);
        var secondSize = new FileInfo(zipPath).Length;

        Assert.True(secondSize > firstSize);
    }

    [Fact]
    public void ArchiveAll_EmptyWorkspace_CreatesZipWithOnlyManifest()
    {
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        Assert.True(File.Exists(zipPath));
        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Empty(entries); // manifest is filtered out by ListArchiveContents
    }

    [Fact]
    public void ArchiveAll_NonexistentWorkspace_Throws()
    {
        var badWorkspace = new WorkspaceService(Path.Combine(_tempDir, "nonexistent"));
        var archive = new ArchiveService(badWorkspace);

        Assert.Throws<DirectoryNotFoundException>(() =>
            archive.ArchiveAll(GetZipPath("bad")));
    }

    // ========================================================================
    // ArchiveSelected
    // ========================================================================

    [Fact]
    public void ArchiveSelected_SingleFile_ArchivesFile()
    {
        var filePath = CreateTestFile("R1/Program/test.src", "content");

        var zipPath = GetZipPath();
        _archive.ArchiveSelected(filePath, zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Single(entries);
        Assert.Equal("R1/Program/test.src", entries[0].RelativePath);
    }

    [Fact]
    public void ArchiveSelected_SrcFile_AutoIncludesDat()
    {
        var srcPath = CreateTestFile("R1/Program/WeldSeam01.src", "DEF WeldSeam01()\nEND");
        CreateTestFile("R1/Program/WeldSeam01.dat", "DEFDAT WeldSeam01\nENDDAT");

        var zipPath = GetZipPath();
        _archive.ArchiveSelected(srcPath, zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Equal(2, entries.Count);
        Assert.Contains(entries, e => e.RelativePath.EndsWith(".src"));
        Assert.Contains(entries, e => e.RelativePath.EndsWith(".dat"));
    }

    [Fact]
    public void ArchiveSelected_SrcWithoutDat_ArchivesOnlySrc()
    {
        var srcPath = CreateTestFile("R1/Program/NoDat.src", "DEF NoDat()\nEND");

        var zipPath = GetZipPath();
        _archive.ArchiveSelected(srcPath, zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Single(entries);
    }

    [Fact]
    public void ArchiveSelected_Directory_ArchivesAllFiles()
    {
        var dirPath = Path.Combine(_workspaceRoot, "R1", "Program", "WeldStation");
        Directory.CreateDirectory(dirPath);
        CreateTestFile("R1/Program/WeldStation/Weld01.src", "content1");
        CreateTestFile("R1/Program/WeldStation/Weld01.dat", "data1");
        CreateTestFile("R1/Program/WeldStation/Weld02.src", "content2");

        var zipPath = GetZipPath();
        _archive.ArchiveSelected(dirPath, zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Equal(3, entries.Count);
    }

    // ========================================================================
    // ListArchiveContents
    // ========================================================================

    [Fact]
    public void ListArchiveContents_ReturnsEntries()
    {
        CreateTestFile("R1/Program/test.src", "content");
        CreateTestFile("Config/system.yaml", "version: 1");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.Equal(2, entries.Count);
        Assert.All(entries, e => Assert.False(string.IsNullOrEmpty(e.RelativePath)));
        Assert.All(entries, e => Assert.True(e.DisplayPath.StartsWith("KRC:\\")));
    }

    [Fact]
    public void ListArchiveContents_ExcludesManifest()
    {
        CreateTestFile("R1/Program/test.src", "content");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.DoesNotContain(entries, e => e.RelativePath == "_archive_manifest.json");
    }

    [Fact]
    public void ListArchiveContents_SortedByPath()
    {
        CreateTestFile("R1/Program/z_last.src", "c");
        CreateTestFile("Config/a_first.yaml", "a");
        CreateTestFile("R1/Program/m_middle.src", "b");

        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        for (int i = 1; i < entries.Count; i++)
        {
            Assert.True(string.Compare(entries[i].RelativePath, entries[i - 1].RelativePath, StringComparison.Ordinal) >= 0);
        }
    }

    // ========================================================================
    // RestoreAll
    // ========================================================================

    [Fact]
    public void RestoreAll_RestoresFilesToWorkspace()
    {
        CreateTestFile("R1/Program/test.src", "original");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Delete file from workspace
        File.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "test.src"));
        Assert.False(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));

        // Restore
        var result = _archive.RestoreAll(zipPath);
        Assert.Equal(1, result.Restored);
        Assert.True(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));
        Assert.Equal("original", File.ReadAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));
    }

    [Fact]
    public void RestoreAll_Overwrite_OverwritesExisting()
    {
        CreateTestFile("R1/Program/test.src", "original");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Modify file
        File.WriteAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src"), "modified");

        // Restore with overwrite
        var result = _archive.RestoreAll(zipPath, ConflictResolution.Overwrite);
        Assert.Equal(1, result.Restored);
        Assert.Equal(1, result.Overwritten);
        Assert.Equal("original", File.ReadAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));
    }

    [Fact]
    public void RestoreAll_Skip_SkipsExistingFiles()
    {
        CreateTestFile("R1/Program/test.src", "original");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Modify file
        File.WriteAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src"), "modified");

        // Restore with skip
        var result = _archive.RestoreAll(zipPath, ConflictResolution.Skip);
        Assert.Equal(0, result.Restored);
        Assert.Equal(1, result.Skipped);
        Assert.Equal("modified", File.ReadAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));
    }

    [Fact]
    public void RestoreAll_Rename_CreatesRenamedCopy()
    {
        CreateTestFile("R1/Program/test.src", "original");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // File still exists — restore with rename
        var result = _archive.RestoreAll(zipPath, ConflictResolution.Rename);
        Assert.Equal(1, result.Restored);

        // Original unchanged
        Assert.Equal("original", File.ReadAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src")));
        // Renamed copy exists
        Assert.True(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "test_restored1.src")));
    }

    [Fact]
    public void RestoreAll_CreatesDirectories()
    {
        CreateTestFile("R1/Program/SubDir/nested.src", "nested content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Delete entire directory
        Directory.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "SubDir"), true);

        // Restore
        var result = _archive.RestoreAll(zipPath);
        Assert.True(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "SubDir", "nested.src")));
    }

    // ========================================================================
    // RestoreSelected
    // ========================================================================

    [Fact]
    public void RestoreSelected_OnlySelectedFiles()
    {
        CreateTestFile("R1/Program/keep.src", "keep");
        CreateTestFile("R1/Program/skip.src", "skip");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Delete both
        File.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "keep.src"));
        File.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "skip.src"));

        // Restore only "keep.src"
        var result = _archive.RestoreSelected(zipPath, new[] { "R1/Program/keep.src" });
        Assert.Equal(1, result.Restored);
        Assert.True(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "keep.src")));
        Assert.False(File.Exists(Path.Combine(_workspaceRoot, "R1", "Program", "skip.src")));
    }

    [Fact]
    public void RestoreSelected_CaseInsensitiveMatch()
    {
        CreateTestFile("R1/Program/Test.src", "content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        File.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "Test.src"));

        // Use different case
        var result = _archive.RestoreSelected(zipPath, new[] { "r1/program/test.src" });
        Assert.Equal(1, result.Restored);
    }

    [Fact]
    public void RestoreSelected_EmptySelection_RestoresNothing()
    {
        CreateTestFile("R1/Program/test.src", "content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var result = _archive.RestoreSelected(zipPath, Array.Empty<string>());
        Assert.Equal(0, result.Restored);
        Assert.Equal(0, result.Total);
    }

    // ========================================================================
    // CompareWithWorkspace
    // ========================================================================

    [Fact]
    public void Compare_IdenticalFiles_StatusIdentical()
    {
        CreateTestFile("R1/Program/test.src", "same content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var results = _archive.CompareWithWorkspace(zipPath);
        var entry = results.First(r => r.RelativePath == "R1/Program/test.src");
        Assert.Equal(CompareStatus.Identical, entry.Status);
    }

    [Fact]
    public void Compare_ModifiedFile_StatusDifferent()
    {
        CreateTestFile("R1/Program/test.src", "original content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Modify local file (different size)
        File.WriteAllText(Path.Combine(_workspaceRoot, "R1", "Program", "test.src"), "modified content - longer");

        var results = _archive.CompareWithWorkspace(zipPath);
        var entry = results.First(r => r.RelativePath == "R1/Program/test.src");
        Assert.Equal(CompareStatus.Different, entry.Status);
    }

    [Fact]
    public void Compare_DeletedLocalFile_StatusOnlyInArchive()
    {
        CreateTestFile("R1/Program/test.src", "content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Delete local
        File.Delete(Path.Combine(_workspaceRoot, "R1", "Program", "test.src"));

        var results = _archive.CompareWithWorkspace(zipPath);
        var entry = results.First(r => r.RelativePath == "R1/Program/test.src");
        Assert.Equal(CompareStatus.OnlyInArchive, entry.Status);
    }

    [Fact]
    public void Compare_NewLocalFile_StatusOnlyLocal()
    {
        CreateTestFile("R1/Program/archived.src", "archived");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Add new local file
        CreateTestFile("R1/Program/new_local.src", "new file");

        var results = _archive.CompareWithWorkspace(zipPath);
        var newFile = results.FirstOrDefault(r => r.RelativePath == "R1/Program/new_local.src");
        Assert.NotNull(newFile);
        Assert.Equal(CompareStatus.OnlyLocal, newFile.Status);
    }

    [Fact]
    public void Compare_ExcludesLogAndCatalog_FromOnlyLocal()
    {
        CreateTestFile("R1/Program/test.src", "content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        // Add files in Log and Catalog
        CreateTestFile("Log/debug.log", "log data");
        CreateTestFile("Catalog/robot/config.yaml", "catalog data");

        var results = _archive.CompareWithWorkspace(zipPath);
        Assert.DoesNotContain(results, r => r.RelativePath.StartsWith("Log/", StringComparison.OrdinalIgnoreCase));
        Assert.DoesNotContain(results, r => r.RelativePath.StartsWith("Catalog/", StringComparison.OrdinalIgnoreCase));
    }

    [Fact]
    public void Compare_ResultsSortedByPath()
    {
        CreateTestFile("R1/Program/z_file.src", "z");
        CreateTestFile("Config/a_file.yaml", "a");
        CreateTestFile("R1/Program/m_file.src", "m");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var results = _archive.CompareWithWorkspace(zipPath);
        for (int i = 1; i < results.Count; i++)
        {
            Assert.True(string.Compare(results[i].RelativePath, results[i - 1].RelativePath, StringComparison.Ordinal) >= 0);
        }
    }

    // ========================================================================
    // RestoreResult model
    // ========================================================================

    [Fact]
    public void RestoreResult_Total_IsSumOfRestoredAndSkipped()
    {
        var result = new RestoreResult { Restored = 3, Skipped = 2, Overwritten = 1 };
        Assert.Equal(5, result.Total);
    }

    // ========================================================================
    // ArchiveEntry model
    // ========================================================================

    [Fact]
    public void ArchiveEntry_DisplayPath_HasKrcPrefix()
    {
        CreateTestFile("R1/Program/test.src", "content");
        var zipPath = GetZipPath();
        _archive.ArchiveAll(zipPath);

        var entries = _archive.ListArchiveContents(zipPath);
        Assert.All(entries, e => Assert.StartsWith("KRC:\\", e.DisplayPath));
    }

    [Fact]
    public void ArchiveEntry_IsSelectedDefaultTrue()
    {
        var entry = new ArchiveEntry();
        Assert.True(entry.IsSelected);
    }
}
