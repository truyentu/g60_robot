/**
 * @file RobotPackageLoader.hpp
 * @brief Robot Package Loader - loads robot packages from directory or ZIP
 *
 * Part of Phase 8: Virtual Simulation (IMPL_P8_01)
 */

#pragma once

#include "RobotPackageSchema.hpp"
#include <optional>
#include <filesystem>
#include <vector>

namespace robot_controller {
namespace config {

/**
 * Robot Package Loader
 *
 * Loads robot packages from:
 * - Directory containing robot.yaml and meshes/
 * - ZIP file (.robotpkg.zip)
 * - Built-in library
 */
class RobotPackageLoader {
public:
    /**
     * Load robot package from directory
     * @param packagePath Path to robot package directory (containing robot.yaml)
     * @return RobotPackage if successful
     */
    static std::optional<RobotPackage> loadFromDirectory(
        const std::filesystem::path& packagePath);

    /**
     * Load robot package from ZIP file
     * @param zipPath Path to .robotpkg.zip file
     * @return RobotPackage if successful
     */
    static std::optional<RobotPackage> loadFromZip(
        const std::filesystem::path& zipPath);

    /**
     * Validate robot package
     * @param package Package to validate
     * @return Error message if invalid, empty string if valid
     */
    static std::string validate(const RobotPackage& package);

    /**
     * Get list of built-in robot packages
     * @return Vector of package info (lightweight)
     */
    static std::vector<RobotPackageInfo> getBuiltInPackages();

    /**
     * Get list of built-in package names
     * @return Vector of package names
     */
    static std::vector<std::string> getBuiltInPackageNames();

    /**
     * Load built-in robot package by name
     * @param name Package name (e.g., "kuka_kr6_r900")
     * @return RobotPackage if found
     */
    static std::optional<RobotPackage> loadBuiltIn(const std::string& name);

    /**
     * Set the robots library directory
     * @param path Path to robots library directory
     */
    static void setLibraryPath(const std::filesystem::path& path);

    /**
     * Get the robots library directory
     */
    static std::filesystem::path getLibraryPath();

    /**
     * Reload packages from library directory
     * Call this after adding new packages to refresh the list
     */
    static void reload();

private:
    /**
     * Parse robot.yaml file
     */
    static bool parseRobotYaml(const std::filesystem::path& yamlPath,
                               RobotPackage& package);

    /**
     * Parse joint definition from YAML node
     */
    static bool parseJoint(const std::string& jointYaml,
                          JointDefinition& joint);

    /**
     * Default robots library path
     */
    static std::filesystem::path s_libraryPath;
};

} // namespace config
} // namespace robot_controller
