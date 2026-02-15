using Serilog;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Text.Json;

namespace RobotController.UI.Services;

/// <summary>
/// ZIP-based archive and restore for KUKA-style workspace backup.
/// Preserves KRC:\ folder structure in archive.
/// </summary>
public class ArchiveService
{
    private readonly WorkspaceService _workspace;

    public ArchiveService(WorkspaceService workspace)
    {
        _workspace = workspace;
    }

    /// <summary>Archive entire workspace to ZIP file</summary>
    public void ArchiveAll(string destinationZipPath)
    {
        var root = _workspace.WorkspaceRoot;
        if (!Directory.Exists(root))
            throw new DirectoryNotFoundException($"Workspace not found: {root}");

        if (File.Exists(destinationZipPath))
            File.Delete(destinationZipPath);

        int fileCount = 0;
        using var zip = ZipFile.Open(destinationZipPath, ZipArchiveMode.Create);

        foreach (var filePath in Directory.EnumerateFiles(root, "*", SearchOption.AllDirectories))
        {
            var relativePath = Path.GetRelativePath(root, filePath).Replace('\\', '/');
            if (relativePath.StartsWith("Log/", StringComparison.OrdinalIgnoreCase)) continue;
            if (relativePath.StartsWith("Catalog/", StringComparison.OrdinalIgnoreCase)) continue;

            zip.CreateEntryFromFile(filePath, relativePath, CompressionLevel.Optimal);
            fileCount++;
        }

        var manifest = new ArchiveManifest
        {
            Timestamp = DateTime.UtcNow.ToString("o"),
            Version = "1.0",
            FileCount = fileCount,
            Source = _workspace.WorkspaceRoot
        };
        var manifestJson = JsonSerializer.Serialize(manifest, new JsonSerializerOptions { WriteIndented = true });
        var manifestEntry = zip.CreateEntry("_archive_manifest.json");
        using (var writer = new StreamWriter(manifestEntry.Open()))
        {
            writer.Write(manifestJson);
        }

        Log.Information("Archive created: {Path} ({Count} files)", destinationZipPath, fileCount);
    }

    /// <summary>Archive selected directory or file to ZIP</summary>
    public void ArchiveSelected(string sourcePath, string destinationZipPath)
    {
        var root = _workspace.WorkspaceRoot;

        if (File.Exists(destinationZipPath))
            File.Delete(destinationZipPath);

        int fileCount = 0;
        using var zip = ZipFile.Open(destinationZipPath, ZipArchiveMode.Create);

        if (Directory.Exists(sourcePath))
        {
            foreach (var filePath in Directory.EnumerateFiles(sourcePath, "*", SearchOption.AllDirectories))
            {
                var relativePath = Path.GetRelativePath(root, filePath).Replace('\\', '/');
                zip.CreateEntryFromFile(filePath, relativePath, CompressionLevel.Optimal);
                fileCount++;
            }
        }
        else if (File.Exists(sourcePath))
        {
            var relativePath = Path.GetRelativePath(root, sourcePath).Replace('\\', '/');
            zip.CreateEntryFromFile(sourcePath, relativePath, CompressionLevel.Optimal);
            fileCount++;

            if (Path.GetExtension(sourcePath).Equals(".src", StringComparison.OrdinalIgnoreCase))
            {
                var datPath = Path.ChangeExtension(sourcePath, ".dat");
                if (File.Exists(datPath))
                {
                    var datRelative = Path.GetRelativePath(root, datPath).Replace('\\', '/');
                    zip.CreateEntryFromFile(datPath, datRelative, CompressionLevel.Optimal);
                    fileCount++;
                }
            }
        }

        var manifest = new ArchiveManifest
        {
            Timestamp = DateTime.UtcNow.ToString("o"),
            Version = "1.0",
            FileCount = fileCount,
            Source = sourcePath
        };
        var manifestJson = JsonSerializer.Serialize(manifest, new JsonSerializerOptions { WriteIndented = true });
        var manifestEntry = zip.CreateEntry("_archive_manifest.json");
        using (var writer = new StreamWriter(manifestEntry.Open()))
        {
            writer.Write(manifestJson);
        }

        Log.Information("Archive (selected) created: {Path} ({Count} files)", destinationZipPath, fileCount);
    }

    /// <summary>List contents of an archive ZIP</summary>
    public List<ArchiveEntry> ListArchiveContents(string zipPath)
    {
        var entries = new List<ArchiveEntry>();
        using var zip = ZipFile.OpenRead(zipPath);

        foreach (var entry in zip.Entries)
        {
            if (entry.FullName == "_archive_manifest.json") continue;
            if (string.IsNullOrEmpty(entry.Name)) continue;

            entries.Add(new ArchiveEntry
            {
                RelativePath = entry.FullName,
                DisplayPath = "KRC:\\" + entry.FullName.Replace('/', '\\'),
                Size = entry.Length,
                CompressedSize = entry.CompressedLength,
                LastModified = entry.LastWriteTime.DateTime
            });
        }

        return entries.OrderBy(e => e.RelativePath).ToList();
    }

    /// <summary>Restore all files from archive to workspace</summary>
    public RestoreResult RestoreAll(string zipPath, ConflictResolution conflictMode = ConflictResolution.Overwrite)
    {
        var root = _workspace.WorkspaceRoot;
        var result = new RestoreResult();

        using var zip = ZipFile.OpenRead(zipPath);

        foreach (var entry in zip.Entries)
        {
            if (entry.FullName == "_archive_manifest.json") continue;
            if (string.IsNullOrEmpty(entry.Name)) continue;

            var destPath = Path.Combine(root, entry.FullName.Replace('/', '\\'));
            var destDir = Path.GetDirectoryName(destPath)!;

            Directory.CreateDirectory(destDir);

            if (File.Exists(destPath))
            {
                switch (conflictMode)
                {
                    case ConflictResolution.Skip:
                        result.Skipped++;
                        continue;
                    case ConflictResolution.Rename:
                        destPath = GetUniqueFileName(destPath);
                        break;
                    case ConflictResolution.Overwrite:
                    default:
                        result.Overwritten++;
                        break;
                }
            }

            entry.ExtractToFile(destPath, overwrite: true);
            result.Restored++;
        }

        Log.Information("Restore complete: {Restored} restored, {Overwritten} overwritten, {Skipped} skipped",
            result.Restored, result.Overwritten, result.Skipped);

        return result;
    }

    /// <summary>Restore selected entries from archive</summary>
    public RestoreResult RestoreSelected(string zipPath, IEnumerable<string> selectedRelativePaths,
        ConflictResolution conflictMode = ConflictResolution.Overwrite)
    {
        var root = _workspace.WorkspaceRoot;
        var result = new RestoreResult();
        var selectedSet = new HashSet<string>(selectedRelativePaths, StringComparer.OrdinalIgnoreCase);

        using var zip = ZipFile.OpenRead(zipPath);

        foreach (var entry in zip.Entries)
        {
            if (!selectedSet.Contains(entry.FullName)) continue;
            if (string.IsNullOrEmpty(entry.Name)) continue;

            var destPath = Path.Combine(root, entry.FullName.Replace('/', '\\'));
            var destDir = Path.GetDirectoryName(destPath)!;

            Directory.CreateDirectory(destDir);

            if (File.Exists(destPath))
            {
                switch (conflictMode)
                {
                    case ConflictResolution.Skip:
                        result.Skipped++;
                        continue;
                    case ConflictResolution.Rename:
                        destPath = GetUniqueFileName(destPath);
                        break;
                    case ConflictResolution.Overwrite:
                    default:
                        result.Overwritten++;
                        break;
                }
            }

            entry.ExtractToFile(destPath, overwrite: true);
            result.Restored++;
        }

        return result;
    }

    /// <summary>Compare archive contents with current workspace</summary>
    public List<CompareResult> CompareWithWorkspace(string zipPath)
    {
        var root = _workspace.WorkspaceRoot;
        var results = new List<CompareResult>();

        using var zip = ZipFile.OpenRead(zipPath);

        foreach (var entry in zip.Entries)
        {
            if (entry.FullName == "_archive_manifest.json") continue;
            if (string.IsNullOrEmpty(entry.Name)) continue;

            var localPath = Path.Combine(root, entry.FullName.Replace('/', '\\'));

            if (!File.Exists(localPath))
            {
                results.Add(new CompareResult
                {
                    RelativePath = entry.FullName,
                    Status = CompareStatus.OnlyInArchive,
                    ArchiveSize = entry.Length,
                    ArchiveDate = entry.LastWriteTime.DateTime
                });
            }
            else
            {
                var localInfo = new FileInfo(localPath);
                bool sameSize = localInfo.Length == entry.Length;
                bool sameDate = Math.Abs((localInfo.LastWriteTime - entry.LastWriteTime.DateTime).TotalSeconds) < 2;

                results.Add(new CompareResult
                {
                    RelativePath = entry.FullName,
                    Status = (sameSize && sameDate) ? CompareStatus.Identical : CompareStatus.Different,
                    ArchiveSize = entry.Length,
                    ArchiveDate = entry.LastWriteTime.DateTime,
                    LocalSize = localInfo.Length,
                    LocalDate = localInfo.LastWriteTime
                });
            }
        }

        foreach (var filePath in Directory.EnumerateFiles(root, "*", SearchOption.AllDirectories))
        {
            var relativePath = Path.GetRelativePath(root, filePath).Replace('\\', '/');
            if (relativePath.StartsWith("Log/", StringComparison.OrdinalIgnoreCase)) continue;
            if (relativePath.StartsWith("Catalog/", StringComparison.OrdinalIgnoreCase)) continue;

            if (!results.Any(r => r.RelativePath.Equals(relativePath, StringComparison.OrdinalIgnoreCase)))
            {
                var localInfo = new FileInfo(filePath);
                results.Add(new CompareResult
                {
                    RelativePath = relativePath,
                    Status = CompareStatus.OnlyLocal,
                    LocalSize = localInfo.Length,
                    LocalDate = localInfo.LastWriteTime
                });
            }
        }

        return results.OrderBy(r => r.RelativePath).ToList();
    }

    private static string GetUniqueFileName(string path)
    {
        var dir = Path.GetDirectoryName(path)!;
        var nameNoExt = Path.GetFileNameWithoutExtension(path);
        var ext = Path.GetExtension(path);
        int counter = 1;
        string newPath;
        do
        {
            newPath = Path.Combine(dir, $"{nameNoExt}_restored{counter}{ext}");
            counter++;
        } while (File.Exists(newPath));
        return newPath;
    }
}

// ===== Data Models =====

public class ArchiveManifest
{
    public string Timestamp { get; set; } = "";
    public string Version { get; set; } = "";
    public int FileCount { get; set; }
    public string Source { get; set; } = "";
}

public class ArchiveEntry
{
    public string RelativePath { get; set; } = "";
    public string DisplayPath { get; set; } = "";
    public long Size { get; set; }
    public long CompressedSize { get; set; }
    public DateTime LastModified { get; set; }
    public bool IsSelected { get; set; } = true;
}

public class RestoreResult
{
    public int Restored { get; set; }
    public int Overwritten { get; set; }
    public int Skipped { get; set; }
    public int Total => Restored + Skipped;
}

public enum ConflictResolution
{
    Overwrite,
    Skip,
    Rename
}

public class CompareResult
{
    public string RelativePath { get; set; } = "";
    public CompareStatus Status { get; set; }
    public long ArchiveSize { get; set; }
    public DateTime? ArchiveDate { get; set; }
    public long LocalSize { get; set; }
    public DateTime? LocalDate { get; set; }
}

public enum CompareStatus
{
    Identical,
    Different,
    OnlyInArchive,
    OnlyLocal
}
