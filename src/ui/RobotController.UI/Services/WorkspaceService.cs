using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using RobotController.UI.Models;
using Serilog;

namespace RobotController.UI.Services;

public class WorkspaceService
{
    private const string DisplayPrefix = "KRC:\\";

    public string WorkspaceRoot { get; }
    public string TemplatesDir { get; }

    // Path helpers — KUKA-style structure under VirtualRoot
    public string ProgramsDir => Path.Combine(WorkspaceRoot, "R1", "Program");
    public string SystemDir => Path.Combine(WorkspaceRoot, "R1", "System");
    public string MadaDir => Path.Combine(WorkspaceRoot, "R1", "Mada");
    public string ToolsDir => Path.Combine(WorkspaceRoot, "Tools");
    public string CatalogDir => Path.Combine(WorkspaceRoot, "Catalog");
    public string StationDir => Path.Combine(WorkspaceRoot, "Station");

    // Legacy paths used by other services (Config, Frames, Log)
    public string ConfigDir => Path.Combine(WorkspaceRoot, "Config");
    public string FramesDir => Path.Combine(WorkspaceRoot, "Frames");
    public string LogDir => Path.Combine(WorkspaceRoot, "Log");

    public WorkspaceService(string? overridePath = null)
    {
        var baseDir = AppDomain.CurrentDomain.BaseDirectory;

        // Resolve Assets dir (contains VirtualRoot and Templates)
        var assetsDir = ResolveAssetsDir(baseDir);

        if (!string.IsNullOrWhiteSpace(overridePath))
        {
            WorkspaceRoot = Path.GetFullPath(overridePath);
        }
        else
        {
            // Try environment variable
            var envPath = Environment.GetEnvironmentVariable("ROBOT_WORKSPACE");
            if (!string.IsNullOrWhiteSpace(envPath) && Directory.Exists(envPath))
            {
                WorkspaceRoot = Path.GetFullPath(envPath);
            }
            else
            {
                // Default: Assets/VirtualRoot/ inside UI project
                WorkspaceRoot = Path.Combine(assetsDir, "VirtualRoot");
            }
        }

        TemplatesDir = Path.Combine(assetsDir, "Templates");
        Log.Information("[Workspace] Root: {Root}", WorkspaceRoot);
        Log.Information("[Workspace] Templates: {Templates}", TemplatesDir);
    }

    private static string ResolveAssetsDir(string baseDir)
    {
        // Try navigating from bin/Debug/net8.0-windows/ to project source
        // bin/Debug/net8.0-windows/ -> ../../.. -> project root -> Assets/
        var candidate = Path.GetFullPath(Path.Combine(baseDir, "..", "..", ".."));
        var assetsPath = Path.Combine(candidate, "Assets");
        if (Directory.Exists(assetsPath))
            return assetsPath;

        // Fallback: Assets/ beside executable
        var fallback = Path.Combine(baseDir, "Assets");
        Directory.CreateDirectory(fallback);
        return fallback;
    }

    // ===== Display Path Conversion =====

    public string ToDisplayPath(string realPath)
    {
        var fullReal = Path.GetFullPath(realPath);
        var fullRoot = Path.GetFullPath(WorkspaceRoot);

        if (fullReal.StartsWith(fullRoot, StringComparison.OrdinalIgnoreCase))
        {
            var relative = fullReal.Substring(fullRoot.Length)
                .TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            return DisplayPrefix + relative.Replace('/', '\\');
        }

        return fullReal;
    }

    public string FromDisplayPath(string displayPath)
    {
        if (displayPath.StartsWith(DisplayPrefix, StringComparison.OrdinalIgnoreCase))
        {
            var relative = displayPath.Substring(DisplayPrefix.Length);
            return Path.Combine(WorkspaceRoot, relative.Replace('\\', Path.DirectorySeparatorChar));
        }

        return displayPath;
    }

    // ===== Workspace Structure =====

    public void EnsureWorkspaceStructure()
    {
        var dirs = new[]
        {
            ProgramsDir, SystemDir, MadaDir,
            Path.Combine(MadaDir, "meshes", "visual"),
            ConfigDir, ToolsDir, FramesDir, CatalogDir, StationDir, LogDir
        };

        foreach (var dir in dirs)
        {
            Directory.CreateDirectory(dir);
        }

        // Seed default data from legacy workspace/ if available
        SeedFromLegacyWorkspace();
    }

    private void SeedFromLegacyWorkspace()
    {
        // Find legacy workspace/ at project root for backward compatibility
        var baseDir = AppDomain.CurrentDomain.BaseDirectory;
        var projectRoot = Path.GetFullPath(Path.Combine(baseDir, "..", "..", "..", "..", "..", "..", ".."));
        var legacyWs = Path.Combine(projectRoot, "workspace");

        if (!Directory.Exists(legacyWs)) return;

        // Seed Tools
        var legacyTools = Path.Combine(legacyWs, "Tools");
        if (Directory.Exists(legacyTools))
        {
            foreach (var file in Directory.GetFiles(legacyTools, "*.yaml"))
            {
                var dest = Path.Combine(ToolsDir, Path.GetFileName(file));
                if (!File.Exists(dest))
                {
                    File.Copy(file, dest, overwrite: false);
                    Log.Debug("[Workspace] Seeded tool from legacy: {Name}", Path.GetFileName(file));
                }
            }
            // Also copy tool meshes
            var legacyMeshes = Path.Combine(legacyTools, "meshes");
            if (Directory.Exists(legacyMeshes))
            {
                var destMeshes = Path.Combine(ToolsDir, "meshes");
                CopyDirectoryRecursive(legacyMeshes, destMeshes);
            }
        }

        // Seed Catalog
        var legacyCatalog = Path.Combine(legacyWs, "Catalog");
        if (Directory.Exists(legacyCatalog))
        {
            foreach (var dir in Directory.GetDirectories(legacyCatalog))
            {
                var id = Path.GetFileName(dir);
                var dest = Path.Combine(CatalogDir, id);
                if (!Directory.Exists(dest))
                {
                    CopyDirectoryRecursive(dir, dest);
                    Log.Debug("[Workspace] Seeded catalog from legacy: {Id}", id);
                }
            }
        }

        // Seed Station
        var legacyStation = Path.Combine(legacyWs, "Station", "station.json");
        var destStation = Path.Combine(StationDir, "station.json");
        if (File.Exists(legacyStation) && !File.Exists(destStation))
        {
            File.Copy(legacyStation, destStation, overwrite: false);
            Log.Debug("[Workspace] Seeded station.json from legacy");
        }

        // Seed Frames
        var legacyFrames = Path.Combine(legacyWs, "Frames");
        if (Directory.Exists(legacyFrames))
        {
            foreach (var file in Directory.GetFiles(legacyFrames, "*.yaml"))
            {
                var dest = Path.Combine(FramesDir, Path.GetFileName(file));
                if (!File.Exists(dest))
                    File.Copy(file, dest, overwrite: false);
            }
        }
    }

    // ===== Robot Catalog =====

    public void ActivateRobotFromCatalog(string catalogId)
    {
        var catalogDir = Path.Combine(CatalogDir, catalogId);
        var srcYaml = Path.Combine(catalogDir, "robot.yaml");
        if (!File.Exists(srcYaml)) return;

        var destYaml = Path.Combine(MadaDir, "robot.yaml");
        File.Copy(srcYaml, destYaml, overwrite: true);

        var srcMeshes = Path.Combine(catalogDir, "meshes", "visual");
        if (Directory.Exists(srcMeshes))
        {
            var destMeshes = Path.Combine(MadaDir, "meshes", "visual");
            if (Directory.Exists(destMeshes))
            {
                foreach (var f in Directory.GetFiles(destMeshes))
                    File.Delete(f);
            }
            Directory.CreateDirectory(destMeshes);
            foreach (var file in Directory.GetFiles(srcMeshes))
                File.Copy(file, Path.Combine(destMeshes, Path.GetFileName(file)), overwrite: true);
        }
    }

    // ===== Directory Tree =====

    public List<DirectoryNode> GetDirectoryTree()
    {
        var root = new DirectoryNode
        {
            Name = "KRC:",
            FullPath = WorkspaceRoot,
            DisplayPath = "KRC:\\",
            IsExpanded = true
        };

        BuildTreeRecursive(root, WorkspaceRoot);
        return new List<DirectoryNode> { root };
    }

    private void BuildTreeRecursive(DirectoryNode parent, string dirPath)
    {
        try
        {
            var subDirs = Directory.GetDirectories(dirPath)
                .OrderBy(d => Path.GetFileName(d))
                .ToArray();

            foreach (var subDir in subDirs)
            {
                var name = Path.GetFileName(subDir);
                var node = new DirectoryNode
                {
                    Name = name,
                    FullPath = subDir,
                    DisplayPath = ToDisplayPath(subDir)
                };

                BuildTreeRecursive(node, subDir);
                parent.Children.Add(node);
            }
        }
        catch (UnauthorizedAccessException) { }
    }

    // ===== Directory Contents (File Folding) =====

    public List<FileItem> GetDirectoryContents(string dirPath, FilterMode filter)
    {
        var items = new List<FileItem>();
        if (!Directory.Exists(dirPath)) return items;

        // Determine if this directory is read-only (System or Mada)
        bool isReadOnlyDir = IsProtectedDirectory(dirPath);

        // Add subdirectories first
        try
        {
            foreach (var subDir in Directory.GetDirectories(dirPath).OrderBy(d => Path.GetFileName(d)))
            {
                var dirInfo = new DirectoryInfo(subDir);
                items.Add(new FileItem
                {
                    Name = dirInfo.Name,
                    DisplayName = dirInfo.Name,
                    Extension = "",
                    FullPath = subDir,
                    IsDirectory = true,
                    IsComposite = false,
                    IsReadOnly = isReadOnlyDir,
                    Type = FileItemType.Folder,
                    Modified = dirInfo.LastWriteTime,
                    ModifiedDisplay = dirInfo.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture),
                    SizeDisplay = ""
                });
            }
        }
        catch (UnauthorizedAccessException) { }

        // Add files
        try
        {
            var files = Directory.GetFiles(dirPath).OrderBy(f => Path.GetFileName(f)).ToArray();

            if (filter == FilterMode.Module)
                AddModuleFilteredFiles(items, files, isReadOnlyDir);
            else
                AddDetailFilteredFiles(items, files, isReadOnlyDir);
        }
        catch (UnauthorizedAccessException) { }

        return items;
    }

    /// <summary>Check if a directory is protected (System or Mada — read-only in Navigator)</summary>
    private bool IsProtectedDirectory(string dirPath)
    {
        var normalizedDir = Path.GetFullPath(dirPath).TrimEnd('\\', '/').ToLowerInvariant();
        var normalizedSystem = Path.GetFullPath(SystemDir).TrimEnd('\\', '/').ToLowerInvariant();
        var normalizedMada = Path.GetFullPath(MadaDir).TrimEnd('\\', '/').ToLowerInvariant();

        return normalizedDir.StartsWith(normalizedSystem) || normalizedDir.StartsWith(normalizedMada);
    }

    private void AddModuleFilteredFiles(List<FileItem> items, string[] files, bool isReadOnly)
    {
        var processed = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var file in files)
        {
            var ext = Path.GetExtension(file).ToLowerInvariant();
            var nameNoExt = Path.GetFileNameWithoutExtension(file);

            if (ext == ".src")
            {
                if (processed.Contains(nameNoExt)) continue;
                processed.Add(nameNoExt);

                // Module — group .src + .dat as one composite entry
                var fi = new FileInfo(file);
                var datPath = Path.ChangeExtension(file, ".dat");
                bool hasDat = File.Exists(datPath);
                long totalSize = fi.Length;
                if (hasDat)
                    totalSize += new FileInfo(datPath).Length;

                var comment = ExtractComment(file);

                items.Add(new FileItem
                {
                    Name = nameNoExt,
                    DisplayName = nameNoExt,
                    Extension = "module",
                    Comment = comment,
                    FullPath = file,
                    IsDirectory = false,
                    IsComposite = hasDat,
                    IsReadOnly = isReadOnly,
                    Type = FileItemType.Module,
                    SizeBytes = totalSize,
                    SizeDisplay = FormatSize(totalSize),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
            else if (ext == ".dat")
            {
                // Skip if corresponding .src exists (already grouped)
                var srcPath = Path.ChangeExtension(file, ".src");
                if (File.Exists(srcPath)) continue;

                // Standalone .dat file
                var fi = new FileInfo(file);
                items.Add(new FileItem
                {
                    Name = Path.GetFileName(file),
                    DisplayName = Path.GetFileName(file),
                    Extension = ext,
                    FullPath = file,
                    IsDirectory = false,
                    IsComposite = false,
                    IsReadOnly = isReadOnly,
                    Type = FileItemType.DataFile,
                    SizeBytes = fi.Length,
                    SizeDisplay = FormatSize(fi.Length),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
            else
            {
                var fi = new FileInfo(file);
                items.Add(new FileItem
                {
                    Name = Path.GetFileName(file),
                    DisplayName = Path.GetFileName(file),
                    Extension = ext,
                    FullPath = file,
                    IsDirectory = false,
                    IsComposite = false,
                    IsReadOnly = isReadOnly,
                    Type = FileItemType.Other,
                    SizeBytes = fi.Length,
                    SizeDisplay = FormatSize(fi.Length),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
        }
    }

    private void AddDetailFilteredFiles(List<FileItem> items, string[] files, bool isReadOnly)
    {
        foreach (var file in files)
        {
            var fi = new FileInfo(file);
            var ext = fi.Extension.ToLowerInvariant();
            var type = ext switch
            {
                ".src" => FileItemType.SourceFile,
                ".dat" => FileItemType.DataFile,
                _ => FileItemType.Other
            };

            var comment = ext == ".src" ? ExtractComment(file) : "";

            items.Add(new FileItem
            {
                Name = fi.Name,
                DisplayName = fi.Name,
                Extension = ext,
                Comment = comment,
                FullPath = file,
                IsDirectory = false,
                IsComposite = false,
                IsReadOnly = isReadOnly,
                Type = type,
                SizeBytes = fi.Length,
                SizeDisplay = FormatSize(fi.Length),
                Modified = fi.LastWriteTime,
                ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
            });
        }
    }

    // ===== Directory Operations =====

    public void CreateFolder(string parentPath, string name)
    {
        Directory.CreateDirectory(Path.Combine(parentPath, name));
    }

    public void DeleteFileOrFolder(string path)
    {
        if (Directory.Exists(path))
        {
            Directory.Delete(path, recursive: true);
            return;
        }

        if (!File.Exists(path)) return;

        // If deleting a .src, also delete matching .dat (composite delete)
        if (Path.GetExtension(path).Equals(".src", StringComparison.OrdinalIgnoreCase))
        {
            var datPath = Path.ChangeExtension(path, ".dat");
            if (File.Exists(datPath))
                File.Delete(datPath);
        }
        File.Delete(path);
    }

    public void RenameFileOrFolder(string oldPath, string newName)
    {
        if (Directory.Exists(oldPath))
        {
            var parent = Path.GetDirectoryName(oldPath)!;
            Directory.Move(oldPath, Path.Combine(parent, newName));
            return;
        }

        if (!File.Exists(oldPath)) return;

        var dir = Path.GetDirectoryName(oldPath)!;
        var ext = Path.GetExtension(oldPath);
        var oldNameNoExt = Path.GetFileNameWithoutExtension(oldPath);
        var newNameNoExt = Path.GetFileNameWithoutExtension(newName);

        // Composite rename: rename both .src and .dat
        if (ext.Equals(".src", StringComparison.OrdinalIgnoreCase))
        {
            File.Move(oldPath, Path.Combine(dir, newNameNoExt + ".src"));

            var oldDat = Path.Combine(dir, oldNameNoExt + ".dat");
            if (File.Exists(oldDat))
                File.Move(oldDat, Path.Combine(dir, newNameNoExt + ".dat"));
        }
        else
        {
            File.Move(oldPath, Path.Combine(dir, newName));
        }
    }

    public void CopyFileOrFolder(string sourcePath, string destDir)
    {
        if (Directory.Exists(sourcePath))
        {
            var dirName = Path.GetFileName(sourcePath);
            CopyDirectoryRecursive(sourcePath, Path.Combine(destDir, dirName));
            return;
        }

        if (!File.Exists(sourcePath)) return;

        File.Copy(sourcePath, Path.Combine(destDir, Path.GetFileName(sourcePath)), overwrite: false);

        // Composite copy: also copy .dat if copying .src
        if (Path.GetExtension(sourcePath).Equals(".src", StringComparison.OrdinalIgnoreCase))
        {
            var datPath = Path.ChangeExtension(sourcePath, ".dat");
            if (File.Exists(datPath))
                File.Copy(datPath, Path.Combine(destDir, Path.GetFileName(datPath)), overwrite: false);
        }
    }

    public void MoveFileOrFolder(string sourcePath, string destDir)
    {
        if (Directory.Exists(sourcePath))
        {
            var dirName = Path.GetFileName(sourcePath);
            Directory.Move(sourcePath, Path.Combine(destDir, dirName));
            return;
        }

        if (!File.Exists(sourcePath)) return;

        File.Move(sourcePath, Path.Combine(destDir, Path.GetFileName(sourcePath)));

        // Composite move: also move .dat if moving .src
        if (Path.GetExtension(sourcePath).Equals(".src", StringComparison.OrdinalIgnoreCase))
        {
            var datPath = Path.ChangeExtension(sourcePath, ".dat");
            if (File.Exists(datPath))
                File.Move(datPath, Path.Combine(destDir, Path.GetFileName(datPath)));
        }
    }

    private static void CopyDirectoryRecursive(string source, string dest)
    {
        Directory.CreateDirectory(dest);

        foreach (var file in Directory.GetFiles(source))
            File.Copy(file, Path.Combine(dest, Path.GetFileName(file)), overwrite: false);

        foreach (var subDir in Directory.GetDirectories(source))
            CopyDirectoryRecursive(subDir, Path.Combine(dest, Path.GetFileName(subDir)));
    }

    // ===== Program Operations (Template System) =====

    public void CreateProgramFromTemplate(string dirPath, string name, string comment = "")
    {
        var srcTemplatePath = Path.Combine(TemplatesDir, "Module.src");
        var datTemplatePath = Path.Combine(TemplatesDir, "Module.dat");

        string srcContent;
        string datContent;

        if (File.Exists(srcTemplatePath) && File.Exists(datTemplatePath))
        {
            srcContent = File.ReadAllText(srcTemplatePath)
                .Replace("{NAME}", name)
                .Replace("{COMMENT}", string.IsNullOrWhiteSpace(comment) ? "New program" : comment);
            datContent = File.ReadAllText(datTemplatePath)
                .Replace("{NAME}", name);
        }
        else
        {
            // Fallback: generate inline if templates missing
            srcContent = GenerateDefaultSrc(name, comment);
            datContent = GenerateDefaultDat(name);
        }

        File.WriteAllText(Path.Combine(dirPath, name + ".src"), srcContent);
        File.WriteAllText(Path.Combine(dirPath, name + ".dat"), datContent);
        Log.Information("[Workspace] Created program from template: {Name}", name);
    }

    // Keep legacy CreateProgram for backward compatibility
    public void CreateProgram(string dirPath, string name, string? template = null)
    {
        if (template != null)
        {
            File.WriteAllText(Path.Combine(dirPath, name + ".src"), template);
            File.WriteAllText(Path.Combine(dirPath, name + ".dat"), GenerateDefaultDat(name));
        }
        else
        {
            CreateProgramFromTemplate(dirPath, name);
        }
    }

    public string LoadProgramSource(string path)
    {
        return File.Exists(path) ? File.ReadAllText(path) : "";
    }

    public string LoadProgramData(string datPath)
    {
        return File.Exists(datPath) ? File.ReadAllText(datPath) : "";
    }

    public void SaveProgramSource(string path, string content)
    {
        File.WriteAllText(path, content);
    }

    public void SaveProgramData(string path, string content)
    {
        File.WriteAllText(path, content);
    }

    // ===== Helpers =====

    private static string ExtractComment(string srcPath)
    {
        try
        {
            using var reader = new StreamReader(srcPath);
            for (int i = 0; i < 10; i++)
            {
                var line = reader.ReadLine();
                if (line == null) break;
                if (line.StartsWith("&COMMENT", StringComparison.OrdinalIgnoreCase))
                {
                    var idx = line.IndexOf(' ');
                    return idx >= 0 ? line.Substring(idx + 1).Trim() : "";
                }
            }
        }
        catch { }
        return "";
    }

    private static string FormatSize(long bytes)
    {
        if (bytes < 1024) return $"{bytes}B";
        if (bytes < 1024 * 1024) return $"{bytes / 1024.0:F1}K";
        return $"{bytes / (1024.0 * 1024.0):F1}M";
    }

    private static string GenerateDefaultSrc(string name, string comment = "")
    {
        var cmt = string.IsNullOrWhiteSpace(comment) ? "New program" : comment;
        return $"&ACCESS RV\n&REL 1\n&COMMENT {cmt}\n\nDEF {name}()\n  ;FOLD INI\n    ;FOLD BASISTECH INI\n      BAS(#INITMOV, 0)\n    ;ENDFOLD\n  ;ENDFOLD\n\n  ;FOLD PTP HOME Vel=100% DEFAULT\n    PTP HOME Vel=100% DEFAULT\n  ;ENDFOLD\n\nEND\n";
    }

    private static string GenerateDefaultDat(string name)
    {
        return $"DEFDAT {name}\n  ;FOLD EXTERNAL DECLARATIONS; %{{PE}}\n  ;ENDFOLD\n\n  DECL E6POS HOME={{A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0}}\n\nENDDAT\n";
    }

    // ===== Legacy compatibility: SeedCatalogFromSource =====

    public void SeedCatalogFromSource()
    {
        SeedFromLegacyWorkspace();
    }
}
