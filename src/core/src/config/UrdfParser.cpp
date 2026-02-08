/**
 * @file UrdfParser.cpp
 * @brief URDF/Xacro parser implementation
 *
 * Uses simple XML parsing without external dependencies.
 * For production, consider using tinyxml2 or pugixml.
 */

#include "UrdfParser.hpp"
#include "../logging/Logger.hpp"
#include <fstream>
#include <sstream>
#include <regex>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace robot_controller {
namespace config {

namespace {
    // Simple XML element extraction using regex
    // Note: For production, use proper XML parser like tinyxml2

    std::string getAttributeValue(const std::string& element, const std::string& attr) {
        // Pattern: attr\s*=\s*"([^"]*)"
        std::regex pattern(attr + "\\s*=\\s*\"([^\"]*)\"");
        std::smatch match;
        if (std::regex_search(element, match, pattern)) {
            return match[1].str();
        }
        return "";
    }

    std::vector<std::string> findElements(const std::string& xml, const std::string& tag) {
        std::vector<std::string> elements;
        std::smatch match;

        // Pattern 1: Tag with attributes - <tag attr="val">content</tag>
        std::string pattern_str = "<" + tag + "\\s[^>]*(?:/>|>[\\s\\S]*?<\\/" + tag + ">)";
        std::regex pattern(pattern_str);

        auto begin = std::sregex_iterator(xml.begin(), xml.end(), pattern);
        auto end = std::sregex_iterator();

        for (auto it = begin; it != end; ++it) {
            elements.push_back(it->str());
        }

        // Pattern 2: Tag without attributes - <tag>content</tag>
        // Only search if we didn't find with attributes pattern
        if (elements.empty()) {
            pattern_str = "<" + tag + ">[\\s\\S]*?<\\/" + tag + ">";
            pattern = std::regex(pattern_str);

            begin = std::sregex_iterator(xml.begin(), xml.end(), pattern);
            for (auto it = begin; it != end; ++it) {
                elements.push_back(it->str());
            }
        }

        return elements;
    }

    std::string findChildElement(const std::string& parent, const std::string& tag) {
        std::smatch match;

        // Pattern 1: Tag with attributes - <tag attr="val">content</tag>
        std::string pattern_str = "<" + tag + "\\s[^>]*(?:/>|>[\\s\\S]*?<\\/" + tag + ">)";
        std::regex pattern(pattern_str);
        if (std::regex_search(parent, match, pattern)) {
            return match[0].str();
        }

        // Pattern 2: Tag without attributes - <tag>content</tag>
        pattern_str = "<" + tag + ">[\\s\\S]*?<\\/" + tag + ">";
        pattern = std::regex(pattern_str);
        if (std::regex_search(parent, match, pattern)) {
            return match[0].str();
        }

        // Pattern 3: Self-closing with attributes - <tag attr="val"/>
        pattern_str = "<" + tag + "\\s[^>]*/>";
        pattern = std::regex(pattern_str);
        if (std::regex_search(parent, match, pattern)) {
            return match[0].str();
        }

        // Pattern 4: Self-closing without attributes - <tag/>
        pattern_str = "<" + tag + "/>";
        pattern = std::regex(pattern_str);
        if (std::regex_search(parent, match, pattern)) {
            return match[0].str();
        }

        return "";
    }

    std::vector<double> parseDoubleArray(const std::string& str) {
        std::vector<double> result;
        std::istringstream iss(str);
        double val;
        while (iss >> val) {
            result.push_back(val);
        }
        return result;
    }
}

UrdfParseResult UrdfParser::parseFile(const std::filesystem::path& filepath) {
    UrdfParseResult result;

    if (!std::filesystem::exists(filepath)) {
        result.error = "File not found: " + filepath.string();
        return result;
    }

    std::ifstream file(filepath);
    if (!file.is_open()) {
        result.error = "Cannot open file: " + filepath.string();
        return result;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    LOG_INFO("Parsing URDF file: {}", filepath.string());
    return parseString(buffer.str());
}

UrdfParseResult UrdfParser::parseString(const std::string& xml_content) {
    UrdfParseResult result;

    try {
        // Parse links
        auto link_elements = findElements(xml_content, "link");
        LOG_DEBUG("Found {} link elements", link_elements.size());

        for (const auto& link_xml : link_elements) {
            UrdfLink link;
            link.name = getAttributeValue(link_xml, "name");

            // Skip empty or special links
            if (link.name.empty()) continue;

            // Remove prefix if present (${prefix})
            // Pattern: \$\{prefix\}
            std::regex prefix_pattern("\\$\\{prefix\\}");
            link.name = std::regex_replace(link.name, prefix_pattern, "");

            // Find visual mesh
            std::string visual = findChildElement(link_xml, "visual");
            if (!visual.empty()) {
                std::string geometry = findChildElement(visual, "geometry");
                if (!geometry.empty()) {
                    std::string mesh = findChildElement(geometry, "mesh");
                    if (!mesh.empty()) {
                        std::string mesh_file = getAttributeValue(mesh, "filename");
                        LOG_DEBUG("Link {} raw mesh_file: '{}'", link.name, mesh_file);
                        link.visual_mesh = extractMeshFilename(mesh_file);
                        LOG_DEBUG("Link {} extracted mesh: '{}'", link.name, link.visual_mesh);
                    } else {
                        LOG_DEBUG("Link {} has geometry but no mesh element", link.name);
                    }
                } else {
                    LOG_DEBUG("Link {} has visual but no geometry element", link.name);
                }

                // Visual origin
                std::string origin = findChildElement(visual, "origin");
                if (!origin.empty()) {
                    parseOrigin(
                        getAttributeValue(origin, "xyz"),
                        getAttributeValue(origin, "rpy"),
                        link.visual_origin_xyz,
                        link.visual_origin_rpy
                    );
                }
            } else {
                LOG_DEBUG("Link {} has no visual element", link.name);
            }

            result.model.links.push_back(link);
            LOG_DEBUG("Parsed link: {} with mesh: {}", link.name, link.visual_mesh);
        }

        // Parse joints
        auto joint_elements = findElements(xml_content, "joint");
        LOG_DEBUG("Found {} joint elements", joint_elements.size());

        for (const auto& joint_xml : joint_elements) {
            UrdfJoint joint;
            joint.name = getAttributeValue(joint_xml, "name");
            joint.type = getAttributeValue(joint_xml, "type");

            if (joint.name.empty()) continue;

            // Remove prefix
            std::regex prefix_pattern("\\$\\{prefix\\}");
            joint.name = std::regex_replace(joint.name, prefix_pattern, "");

            // Parse parent/child
            std::string parent = findChildElement(joint_xml, "parent");
            std::string child = findChildElement(joint_xml, "child");
            joint.parent_link = std::regex_replace(getAttributeValue(parent, "link"), prefix_pattern, "");
            joint.child_link = std::regex_replace(getAttributeValue(child, "link"), prefix_pattern, "");

            // Parse origin
            std::string origin = findChildElement(joint_xml, "origin");
            if (!origin.empty()) {
                parseOrigin(
                    getAttributeValue(origin, "xyz"),
                    getAttributeValue(origin, "rpy"),
                    joint.origin_xyz,
                    joint.origin_rpy
                );
            }

            // Parse axis
            std::string axis = findChildElement(joint_xml, "axis");
            if (!axis.empty()) {
                parseAxis(getAttributeValue(axis, "xyz"), joint.axis);
            }

            // Parse limits
            std::string limit = findChildElement(joint_xml, "limit");
            if (!limit.empty()) {
                parseLimit(
                    getAttributeValue(limit, "lower"),
                    getAttributeValue(limit, "upper"),
                    getAttributeValue(limit, "velocity"),
                    getAttributeValue(limit, "effort"),
                    joint
                );
            }

            result.model.joints.push_back(joint);
            LOG_DEBUG("Parsed joint: {} type: {} origin: [{}, {}, {}]",
                joint.name, joint.type,
                joint.origin_xyz[0], joint.origin_xyz[1], joint.origin_xyz[2]);
        }

        // Order joints and find base
        orderJoints(result.model);
        result.model.base_link_name = findBaseLink(result.model);

        result.success = true;
        LOG_INFO("URDF parsing complete: {} links, {} joints",
            result.model.links.size(), result.model.joints.size());

    } catch (const std::exception& e) {
        result.error = std::string("Parse error: ") + e.what();
        LOG_ERROR("URDF parse error: {}", e.what());
    }

    return result;
}

void UrdfParser::parseOrigin(const std::string& xyz_str, const std::string& rpy_str,
                              std::array<double, 3>& xyz, std::array<double, 3>& rpy) {
    if (!xyz_str.empty()) {
        auto vals = parseDoubleArray(xyz_str);
        if (vals.size() >= 3) {
            xyz = {vals[0], vals[1], vals[2]};
        }
    }

    if (!rpy_str.empty()) {
        auto vals = parseDoubleArray(rpy_str);
        if (vals.size() >= 3) {
            rpy = {vals[0], vals[1], vals[2]};
        }
    }
}

void UrdfParser::parseAxis(const std::string& xyz_str, std::array<double, 3>& axis) {
    if (!xyz_str.empty()) {
        auto vals = parseDoubleArray(xyz_str);
        if (vals.size() >= 3) {
            axis = {vals[0], vals[1], vals[2]};
        }
    }
}

void UrdfParser::parseLimit(const std::string& lower, const std::string& upper,
                             const std::string& velocity, const std::string& effort,
                             UrdfJoint& joint) {
    if (!lower.empty()) {
        joint.limit_lower = expandXacroExpression(lower);
    }
    if (!upper.empty()) {
        joint.limit_upper = expandXacroExpression(upper);
    }
    if (!velocity.empty()) {
        joint.limit_velocity = expandXacroExpression(velocity);
    }
    if (!effort.empty()) {
        joint.limit_effort = expandXacroExpression(effort);
    }
}

double UrdfParser::expandXacroExpression(const std::string& expr) {
    // Handle ${radians(X)} pattern
    // Pattern: \$\{radians\(([-\d.]+)\)\}
    std::regex radians_pattern("\\$\\{radians\\(([-\\d.]+)\\)\\}");
    std::smatch match;

    if (std::regex_search(expr, match, radians_pattern)) {
        double degrees = std::stod(match[1].str());
        return degrees * M_PI / 180.0;  // Convert to radians
    }

    // Handle plain numbers
    try {
        return std::stod(expr);
    } catch (...) {
        return 0.0;
    }
}

std::string UrdfParser::extractMeshFilename(const std::string& mesh_path) {
    // Extract filename from various mesh path formats:
    // 1. package://pkg_name/meshes/visual/file.stl
    // 2. package://pkg_name/meshes/gp110/visual/file.stl
    // 3. file:///path/to/file.stl
    // 4. meshes/visual/file.stl

    // Try to find just the filename (last component ending in .stl or .STL)
    std::regex filename_pattern("([^/\\\\]+\\.[sS][tT][lL])\\s*$");
    std::smatch match;

    if (std::regex_search(mesh_path, match, filename_pattern)) {
        return match[1].str();
    }

    // If no match, return empty (will use fallback)
    return "";
}

std::string UrdfParser::findBaseLink(const UrdfModel& model) {
    std::set<std::string> child_links;
    for (const auto& joint : model.joints) {
        child_links.insert(joint.child_link);
    }

    for (const auto& link : model.links) {
        if (child_links.find(link.name) == child_links.end()) {
            // This link is not a child of any joint = base link
            if (link.name.find("base") != std::string::npos) {
                return link.name;
            }
        }
    }

    // Fallback: return first link
    if (!model.links.empty()) {
        return model.links[0].name;
    }
    return "base_link";
}

void UrdfParser::orderJoints(UrdfModel& model) {
    // Build parent-child map
    std::map<std::string, std::string> child_to_parent;
    std::map<std::string, std::string> parent_to_joint;

    for (const auto& joint : model.joints) {
        if (joint.type == "fixed") continue;  // Skip fixed joints
        child_to_parent[joint.child_link] = joint.parent_link;
        parent_to_joint[joint.parent_link] = joint.name;
    }

    // Find joint order by traversing from base
    model.joint_order.clear();
    std::string current_link = findBaseLink(model);

    for (size_t i = 0; i < model.joints.size(); ++i) {
        auto it = parent_to_joint.find(current_link);
        if (it != parent_to_joint.end()) {
            model.joint_order.push_back(it->second);

            // Find next link
            for (const auto& joint : model.joints) {
                if (joint.name == it->second) {
                    current_link = joint.child_link;
                    break;
                }
            }
        }
    }
}

std::string UrdfParser::generateYaml(
    const UrdfModel& model,
    const std::string& robot_name,
    const std::string& manufacturer) {

    std::ostringstream yaml;

    yaml << "# " << robot_name << " Robot Package\n";
    yaml << "# Auto-generated from URDF\n";
    yaml << "# Note: DH parameters need to be filled in manually or calculated\n\n";

    yaml << "name: \"" << robot_name << "\"\n";
    yaml << "manufacturer: \"" << manufacturer << "\"\n";
    yaml << "type: \"6-axis-industrial\"\n";
    yaml << "payload_kg: 0  # TODO: Fill in\n";
    yaml << "reach_mm: 0    # TODO: Fill in\n\n";

    yaml << "kinematics:\n";
    yaml << "  convention: \"modified_dh\"\n";
    yaml << "  joints:\n";

    int joint_num = 1;
    for (const auto& joint_name : model.joint_order) {
        // Find joint data
        const UrdfJoint* joint = nullptr;
        for (const auto& j : model.joints) {
            if (j.name == joint_name) {
                joint = &j;
                break;
            }
        }
        if (!joint || joint->type == "fixed") continue;

        // Find corresponding link
        const UrdfLink* link = nullptr;
        for (const auto& l : model.links) {
            if (l.name == joint->child_link) {
                link = &l;
                break;
            }
        }

        yaml << "    # Joint A" << joint_num << " (" << joint->name << ")\n";
        yaml << "    - name: \"A" << joint_num << "\"\n";
        yaml << "      type: \"" << joint->type << "\"\n";

        // DH parameters (placeholder - need calculation)
        yaml << "      # DH parameters (TODO: Calculate from geometry)\n";
        yaml << "      dh:\n";
        yaml << "        a: 0      # Link length (mm)\n";
        yaml << "        alpha: 0  # Link twist (degrees)\n";
        yaml << "        d: 0      # Link offset (mm)\n";
        yaml << "        theta_offset: 0\n";

        // URDF origin (converted to mm)
        yaml << "      # URDF origin (from URDF file)\n";
        yaml << "      origin:\n";
        yaml << "        xyz: ["
             << metersToMm(joint->origin_xyz[0]) << ", "
             << metersToMm(joint->origin_xyz[1]) << ", "
             << metersToMm(joint->origin_xyz[2]) << "]\n";
        yaml << "        rpy: ["
             << joint->origin_rpy[0] << ", "
             << joint->origin_rpy[1] << ", "
             << joint->origin_rpy[2] << "]\n";

        // Axis
        yaml << "      axis: ["
             << joint->axis[0] << ", "
             << joint->axis[1] << ", "
             << joint->axis[2] << "]\n";

        // Limits (converted to degrees)
        yaml << "      limits:\n";
        yaml << "        min: " << radiansToDegrees(joint->limit_lower) << "\n";
        yaml << "        max: " << radiansToDegrees(joint->limit_upper) << "\n";
        yaml << "        vel_max: " << radiansToDegrees(joint->limit_velocity) << "\n";
        yaml << "        accel_max: 720  # TODO: Fill in\n";

        // Mesh
        if (link && !link->visual_mesh.empty()) {
            yaml << "      mesh:\n";
            yaml << "        visual: \"meshes/visual/" << link->visual_mesh << "\"\n";
        } else {
            yaml << "      mesh:\n";
            yaml << "        visual: \"meshes/visual/link_" << joint_num << ".stl\"\n";
        }

        yaml << "\n";
        joint_num++;
    }

    // Home position
    yaml << "home_position: [0";
    for (int i = 1; i < joint_num - 1; ++i) {
        yaml << ", 0";
    }
    yaml << "]\n\n";

    // Base mesh - find actual base mesh from model
    std::string base_mesh = "base_link.stl";  // Default
    for (const auto& link : model.links) {
        if (link.name == model.base_link_name && !link.visual_mesh.empty()) {
            base_mesh = link.visual_mesh;
            break;
        }
    }

    yaml << "base:\n";
    yaml << "  mesh: \"meshes/visual/" << base_mesh << "\"\n";
    yaml << "  origin:\n";
    yaml << "    x: 0\n";
    yaml << "    y: 0\n";
    yaml << "    z: 0\n";
    yaml << "    rx: 0\n";
    yaml << "    ry: 0\n";
    yaml << "    rz: 0\n\n";

    yaml << "flange:\n";
    yaml << "  offset:\n";
    yaml << "    x: 0\n";
    yaml << "    y: 0\n";
    yaml << "    z: 0\n";

    return yaml.str();
}

} // namespace config
} // namespace robot_controller
