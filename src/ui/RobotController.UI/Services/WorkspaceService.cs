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

    // Path helpers
    public string ProgramsDir => Path.Combine(WorkspaceRoot, "R1", "Programs");
    public string SystemDir => Path.Combine(WorkspaceRoot, "R1", "System");
    public string MadaDir => Path.Combine(WorkspaceRoot, "R1", "Mada");
    public string ConfigDir => Path.Combine(WorkspaceRoot, "Config");
    public string ToolsDir => Path.Combine(WorkspaceRoot, "Tools");
    public string FramesDir => Path.Combine(WorkspaceRoot, "Frames");
    public string CatalogDir => Path.Combine(WorkspaceRoot, "Catalog");
    public string StationDir => Path.Combine(WorkspaceRoot, "Station");
    public string LogDir => Path.Combine(WorkspaceRoot, "Log");

    public WorkspaceService(string? overridePath = null)
    {
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
                // Default: navigate from bin output to project root workspace/
                var baseDir = AppDomain.CurrentDomain.BaseDirectory;
                var projectRoot = Path.GetFullPath(
                    Path.Combine(baseDir, "..", "..", "..", "..", "..", "..", ".."));
                var wsPath = Path.Combine(projectRoot, "workspace");
                if (Directory.Exists(wsPath))
                {
                    WorkspaceRoot = wsPath;
                }
                else
                {
                    // Fallback: workspace/ beside the executable
                    WorkspaceRoot = Path.Combine(baseDir, "workspace");
                }
            }
        }
    }

    /// <summary>
    /// Convert real filesystem path to display path: "KRC:\R1\Programs\..."
    /// </summary>
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

    /// <summary>
    /// Convert display path "KRC:\..." back to real filesystem path
    /// </summary>
    public string FromDisplayPath(string displayPath)
    {
        if (displayPath.StartsWith(DisplayPrefix, StringComparison.OrdinalIgnoreCase))
        {
            var relative = displayPath.Substring(DisplayPrefix.Length);
            return Path.Combine(WorkspaceRoot, relative.Replace('\\', Path.DirectorySeparatorChar));
        }

        return displayPath;
    }

    /// <summary>
    /// Ensure all workspace directories exist, then seed default data if catalog is empty.
    /// </summary>
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

        // Seed catalog from src/config/ if workspace catalog is empty
        SeedCatalogFromSource();
    }

    /// <summary>
    /// Resolve project root from workspace path (workspace/ is at project root)
    /// </summary>
    private string? ResolveProjectRoot()
    {
        // workspace/ is directly under project root
        var candidate = Path.GetFullPath(Path.Combine(WorkspaceRoot, ".."));
        var srcConfig = Path.Combine(candidate, "src", "config", "robots");
        if (Directory.Exists(srcConfig))
        {
            Log.Debug("[Workspace] Project root resolved: {Root}", candidate);
            return candidate;
        }
        Log.Warning("[Workspace] Could not resolve project root from workspace: {Ws}", WorkspaceRoot);
        return null;
    }

    /// <summary>
    /// Seed workspace Catalog, Tools, Frames from src/config/ if not already populated.
    /// Only copies when target files don't exist (guard against overwriting user changes).
    /// </summary>
    public void SeedCatalogFromSource()
    {
        var projectRoot = ResolveProjectRoot();
        if (projectRoot == null)
        {
            Log.Warning("[Workspace] SeedCatalog skipped — project root not found");
            return;
        }

        var srcRobots = Path.Combine(projectRoot, "src", "config", "robots");
        var srcTools = Path.Combine(projectRoot, "src", "config", "tools");
        var srcFrames = Path.Combine(projectRoot, "src", "config", "frames");

        // Seed robot packages into Catalog/
        if (Directory.Exists(srcRobots))
        {
            foreach (var robotDir in Directory.GetDirectories(srcRobots))
            {
                var robotId = Path.GetFileName(robotDir);
                var destDir = Path.Combine(CatalogDir, robotId);
                var destYaml = Path.Combine(destDir, "robot.yaml");

                // Guard: skip if already seeded
                if (File.Exists(destYaml))
                {
                    Log.Debug("[Workspace] Catalog {Id} already seeded, skipping", robotId);
                    continue;
                }

                var srcYaml = Path.Combine(robotDir, "robot.yaml");
                if (!File.Exists(srcYaml)) continue;

                Directory.CreateDirectory(destDir);
                File.Copy(srcYaml, destYaml, overwrite: false);
                Log.Information("[Workspace] Seeded catalog: {Id}", robotId);

                // Copy meshes if they exist
                var srcMeshes = Path.Combine(robotDir, "meshes", "visual");
                if (Directory.Exists(srcMeshes) && Directory.GetFiles(srcMeshes).Length > 0)
                {
                    var destMeshes = Path.Combine(destDir, "meshes", "visual");
                    CopyDirectoryRecursive(srcMeshes, destMeshes);
                    Log.Information("[Workspace] Seeded meshes for {Id}: {Count} files",
                        robotId, Directory.GetFiles(srcMeshes).Length);
                }
            }
        }

        // Seed tools
        if (Directory.Exists(srcTools))
        {
            foreach (var toolFile in Directory.GetFiles(srcTools, "*.yaml"))
            {
                var destFile = Path.Combine(ToolsDir, Path.GetFileName(toolFile));
                if (!File.Exists(destFile))
                {
                    File.Copy(toolFile, destFile, overwrite: false);
                    Log.Information("[Workspace] Seeded tool: {Name}", Path.GetFileName(toolFile));
                }
            }
        }

        // Seed frames
        if (Directory.Exists(srcFrames))
        {
            foreach (var frameFile in Directory.GetFiles(srcFrames, "*.yaml"))
            {
                var destFile = Path.Combine(FramesDir, Path.GetFileName(frameFile));
                if (!File.Exists(destFile))
                {
                    File.Copy(frameFile, destFile, overwrite: false);
                    Log.Information("[Workspace] Seeded frame: {Name}", Path.GetFileName(frameFile));
                }
            }
        }

        // Seed station data from bin/config/station/ if workspace station is placeholder
        SeedStationFromBin();
    }

    /// <summary>
    /// Copy station.json from bin/config/station/ if workspace station is empty placeholder.
    /// </summary>
    private void SeedStationFromBin()
    {
        var wsStation = Path.Combine(StationDir, "station.json");
        var binStation = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "config", "station", "station.json");

        if (!File.Exists(binStation))
        {
            Log.Debug("[Workspace] No bin station.json found, skipping station seed");
            return;
        }

        // If workspace station.json doesn't exist, copy from bin
        if (!File.Exists(wsStation))
        {
            File.Copy(binStation, wsStation, overwrite: false);
            Log.Information("[Workspace] Seeded station.json from bin config");
            return;
        }

        // If workspace station.json exists but has empty objects, replace with bin version
        try
        {
            var wsContent = File.ReadAllText(wsStation);
            // Check if it's a placeholder (has "objects": [] or no "Objects" key with data)
            if (wsContent.Contains("\"objects\": []") || wsContent.Contains("\"Objects\": []") ||
                (!wsContent.Contains("\"MeshPath\"") && !wsContent.Contains("\"meshPath\"")))
            {
                File.Copy(binStation, wsStation, overwrite: true);
                Log.Information("[Workspace] Replaced placeholder station.json with bin version");
            }
            else
            {
                Log.Debug("[Workspace] Station.json already has objects, keeping workspace version");
            }
        }
        catch (Exception ex)
        {
            Log.Warning(ex, "[Workspace] Error checking station.json, keeping existing");
        }
    }

    /// <summary>
    /// Copy robot from Catalog into R1/Mada/ as the active robot.
    /// </summary>
    public void ActivateRobotFromCatalog(string catalogId)
    {
        var catalogDir = Path.Combine(CatalogDir, catalogId);
        var srcYaml = Path.Combine(catalogDir, "robot.yaml");
        if (!File.Exists(srcYaml)) return;

        // Copy robot.yaml → R1/Mada/robot.yaml (overwrite — this IS the active robot)
        var destYaml = Path.Combine(MadaDir, "robot.yaml");
        File.Copy(srcYaml, destYaml, overwrite: true);

        // Copy meshes → R1/Mada/meshes/visual/
        var srcMeshes = Path.Combine(catalogDir, "meshes", "visual");
        if (Directory.Exists(srcMeshes))
        {
            var destMeshes = Path.Combine(MadaDir, "meshes", "visual");
            // Clear old meshes first
            if (Directory.Exists(destMeshes))
            {
                foreach (var f in Directory.GetFiles(destMeshes))
                    File.Delete(f);
            }
            Directory.CreateDirectory(destMeshes);
            foreach (var file in Directory.GetFiles(srcMeshes))
            {
                File.Copy(file, Path.Combine(destMeshes, Path.GetFileName(file)), overwrite: true);
            }
        }
    }

    // ===== Directory Tree =====

    /// <summary>
    /// Build the directory tree starting from workspace root
    /// </summary>
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
        catch (UnauthorizedAccessException)
        {
            // Skip inaccessible directories
        }
    }

    // ===== Directory Contents =====

    /// <summary>
    /// Get contents of a directory for the file list panel
    /// </summary>
    public List<FileItem> GetDirectoryContents(string dirPath, FilterMode filter)
    {
        var items = new List<FileItem>();
        if (!Directory.Exists(dirPath)) return items;

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
            {
                AddModuleFilteredFiles(items, files);
            }
            else
            {
                AddDetailFilteredFiles(items, files);
            }
        }
        catch (UnauthorizedAccessException) { }

        return items;
    }

    private void AddModuleFilteredFiles(List<FileItem> items, string[] files)
    {
        var processed = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var file in files)
        {
            var ext = Path.GetExtension(file).ToLowerInvariant();
            var nameNoExt = Path.GetFileNameWithoutExtension(file);

            if (ext == ".program")
            {
                if (processed.Contains(nameNoExt)) continue;
                processed.Add(nameNoExt);

                // This is a module — group .program + .dat
                var fi = new FileInfo(file);
                var datPath = Path.ChangeExtension(file, ".dat");
                long totalSize = fi.Length;
                if (File.Exists(datPath))
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
                    Type = FileItemType.Module,
                    SizeBytes = totalSize,
                    SizeDisplay = FormatSize(totalSize),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
            else if (ext == ".dat")
            {
                // If corresponding .program exists, this was already grouped
                var progPath = Path.ChangeExtension(file, ".program");
                if (File.Exists(progPath)) continue;

                // Standalone .dat file
                var fi = new FileInfo(file);
                items.Add(new FileItem
                {
                    Name = Path.GetFileName(file),
                    DisplayName = Path.GetFileName(file),
                    Extension = ext,
                    FullPath = file,
                    IsDirectory = false,
                    Type = FileItemType.DataFile,
                    SizeBytes = fi.Length,
                    SizeDisplay = FormatSize(fi.Length),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
            else
            {
                // Other file types
                var fi = new FileInfo(file);
                items.Add(new FileItem
                {
                    Name = Path.GetFileName(file),
                    DisplayName = Path.GetFileName(file),
                    Extension = ext,
                    FullPath = file,
                    IsDirectory = false,
                    Type = FileItemType.Other,
                    SizeBytes = fi.Length,
                    SizeDisplay = FormatSize(fi.Length),
                    Modified = fi.LastWriteTime,
                    ModifiedDisplay = fi.LastWriteTime.ToString("MM/dd HH:mm", CultureInfo.InvariantCulture)
                });
            }
        }
    }

    private void AddDetailFilteredFiles(List<FileItem> items, string[] files)
    {
        foreach (var file in files)
        {
            var fi = new FileInfo(file);
            var ext = fi.Extension.ToLowerInvariant();
            var type = ext switch
            {
                ".program" => FileItemType.ProgramFile,
                ".dat" => FileItemType.DataFile,
                _ => FileItemType.Other
            };

            var comment = ext == ".program" ? ExtractComment(file) : "";

            items.Add(new FileItem
            {
                Name = fi.Name,
                DisplayName = fi.Name,
                Extension = ext,
                Comment = comment,
                FullPath = file,
                IsDirectory = false,
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
        var path = Path.Combine(parentPath, name);
        Directory.CreateDirectory(path);
    }

    public void DeleteFileOrFolder(string path)
    {
        if (Directory.Exists(path))
        {
            Directory.Delete(path, recursive: true);
        }
        else if (File.Exists(path))
        {
            // If deleting a .program, also delete matching .dat
            if (Path.GetExtension(path).Equals(".program", StringComparison.OrdinalIgnoreCase))
            {
                var datPath = Path.ChangeExtension(path, ".dat");
                if (File.Exists(datPath))
                    File.Delete(datPath);
            }
            File.Delete(path);
        }
    }

    public void RenameFileOrFolder(string oldPath, string newName)
    {
        if (Directory.Exists(oldPath))
        {
            var parent = Path.GetDirectoryName(oldPath)!;
            var newPath = Path.Combine(parent, newName);
            Directory.Move(oldPath, newPath);
        }
        else if (File.Exists(oldPath))
        {
            var parent = Path.GetDirectoryName(oldPath)!;
            var ext = Path.GetExtension(oldPath);
            var oldNameNoExt = Path.GetFileNameWithoutExtension(oldPath);

            // Rename both .program and .dat if it's a module
            if (ext.Equals(".program", StringComparison.OrdinalIgnoreCase))
            {
                var newNameNoExt = Path.GetFileNameWithoutExtension(newName);
                File.Move(oldPath, Path.Combine(parent, newNameNoExt + ".program"));

                var oldDat = Path.Combine(parent, oldNameNoExt + ".dat");
                if (File.Exists(oldDat))
                    File.Move(oldDat, Path.Combine(parent, newNameNoExt + ".dat"));
            }
            else
            {
                File.Move(oldPath, Path.Combine(parent, newName));
            }
        }
    }

    public void CopyFileOrFolder(string sourcePath, string destDir)
    {
        if (Directory.Exists(sourcePath))
        {
            var dirName = Path.GetFileName(sourcePath);
            var destPath = Path.Combine(destDir, dirName);
            CopyDirectoryRecursive(sourcePath, destPath);
        }
        else if (File.Exists(sourcePath))
        {
            var fileName = Path.GetFileName(sourcePath);
            File.Copy(sourcePath, Path.Combine(destDir, fileName), overwrite: false);

            // Also copy .dat if copying a .program
            if (Path.GetExtension(sourcePath).Equals(".program", StringComparison.OrdinalIgnoreCase))
            {
                var datPath = Path.ChangeExtension(sourcePath, ".dat");
                if (File.Exists(datPath))
                {
                    var datFileName = Path.GetFileName(datPath);
                    File.Copy(datPath, Path.Combine(destDir, datFileName), overwrite: false);
                }
            }
        }
    }

    private static void CopyDirectoryRecursive(string source, string dest)
    {
        Directory.CreateDirectory(dest);

        foreach (var file in Directory.GetFiles(source))
        {
            File.Copy(file, Path.Combine(dest, Path.GetFileName(file)), overwrite: false);
        }

        foreach (var subDir in Directory.GetDirectories(source))
        {
            CopyDirectoryRecursive(subDir, Path.Combine(dest, Path.GetFileName(subDir)));
        }
    }

    // ===== Program Operations =====

    public void CreateProgram(string dirPath, string name, string? template = null)
    {
        var programPath = Path.Combine(dirPath, name + ".program");
        var datPath = Path.Combine(dirPath, name + ".dat");

        var programContent = template ?? GenerateDefaultProgram(name);
        var datContent = GenerateDefaultDat(name);

        File.WriteAllText(programPath, programContent);
        File.WriteAllText(datPath, datContent);
    }

    public string LoadProgramSource(string programPath)
    {
        return File.Exists(programPath) ? File.ReadAllText(programPath) : "";
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

    private static string ExtractComment(string programPath)
    {
        try
        {
            using var reader = new StreamReader(programPath);
            for (int i = 0; i < 10; i++) // Check first 10 lines
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

    private static string GenerateDefaultProgram(string name)
    {
        return $"""
            &ACCESS RV
            &REL 1
            &COMMENT New program
            &USER default

            DEF {name}()
              ;FOLD INI
                ;FOLD BASISTECH INI
                  BAS(#INITMOV, 0)
                ;ENDFOLD
              ;ENDFOLD

              PTP HOME Vel=100% DEFAULT

              ; TODO: Add motion commands here

              PTP HOME Vel=100% DEFAULT

            END
            """;
    }

    private static string GenerateDefaultDat(string name)
    {
        return $$"""
            DEFDAT {{name}}
              ;FOLD EXTERNAL DECLARATIONS; %{PE}
              ;ENDFOLD

              DECL E6POS HOME={A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 0.0, A6 0.0}

            ENDDAT
            """;
    }
}
