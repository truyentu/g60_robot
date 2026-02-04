#!/usr/bin/env python3
"""
Feature Verification Script for Robot Controller Project
Checks if features exist in codebase and their implementation status.

Usage:
    python verify_features.py [--output report.md] [--json]
"""

import os
import re
import json
import argparse
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from enum import Enum

class Status(Enum):
    MISSING = "❌ Missing"
    STUB = "⚠️ Stub Only"
    PARTIAL = "🔶 Partial"
    IMPLEMENTED = "✅ Implemented"
    UNKNOWN = "❓ Unknown"

@dataclass
class Feature:
    id: str
    name: str
    category: str
    check_type: str  # "file", "class", "function", "pattern"
    check_target: str  # path, class name, function name, or regex pattern
    layer: str  # "core", "ui", "firmware"
    status: Status = Status.UNKNOWN
    details: str = ""
    loc: int = 0  # lines of code

@dataclass
class FeatureCategory:
    name: str
    features: List[Feature] = field(default_factory=list)

# Project root
PROJECT_ROOT = Path(__file__).parent.parent
CORE_SRC = PROJECT_ROOT / "src" / "core" / "src"
CORE_TESTS = PROJECT_ROOT / "src" / "core" / "tests"
UI_SRC = PROJECT_ROOT / "src" / "ui"

# Feature definitions
FEATURE_REGISTRY: List[Feature] = [
    # ============= CORE PLATFORM =============
    # Config System
    Feature("CP001", "ConfigManager", "Core Platform", "file", "config/ConfigManager.cpp", "core"),
    Feature("CP002", "RobotConfig (DH Parameters)", "Core Platform", "class", "RobotConfig", "core"),
    Feature("CP003", "SystemConfig", "Core Platform", "class", "SystemConfig", "core"),

    # IPC
    Feature("CP004", "IpcServer", "Core Platform", "file", "ipc/IpcServer.cpp", "core"),
    Feature("CP005", "REQ-REP Pattern", "Core Platform", "pattern", r"ZMQ_REP|zmq::socket_type::rep", "core"),
    Feature("CP006", "PUB-SUB Pattern", "Core Platform", "pattern", r"ZMQ_PUB|zmq::socket_type::pub", "core"),
    Feature("CP007", "Message Serialization", "Core Platform", "class", "Message", "core"),
    Feature("CP008", "CRC32 Checksum", "Core Platform", "pattern", r"crc32|CRC32", "core"),
    Feature("CP009", "Heartbeat Monitoring", "Core Platform", "pattern", r"heartbeat|Heartbeat", "core"),

    # Logging
    Feature("CP010", "Logger (spdlog)", "Core Platform", "file", "logging/Logger.cpp", "core"),

    # ============= STATE MACHINE =============
    Feature("SM001", "StateMachine", "State Machine", "file", "state/StateMachine.cpp", "core"),
    Feature("SM002", "State: BOOT", "State Machine", "pattern", r"BOOT|Boot", "core"),
    Feature("SM003", "State: ESTOP_ACTIVE", "State Machine", "pattern", r"ESTOP|EStop|E_STOP", "core"),
    Feature("SM004", "State: IDLE", "State Machine", "pattern", r"State::IDLE|IDLE", "core"),
    Feature("SM005", "State: OPERATIONAL", "State Machine", "pattern", r"OPERATIONAL|Operational", "core"),
    Feature("SM006", "Mode: T1 (250mm/s limit)", "State Machine", "pattern", r"T1|250\.?0?.*mm", "core"),
    Feature("SM007", "Mode: T2", "State Machine", "pattern", r"T2|OperatingMode", "core"),
    Feature("SM008", "Mode: AUTO", "State Machine", "pattern", r"AUTO|Automatic", "core"),

    # ============= SAFETY =============
    Feature("SF001", "SafetyMonitor", "Safety", "class", "SafetyMonitor", "core"),
    Feature("SF002", "Dual-Channel Safety", "Safety", "pattern", r"dual.?channel|channel[12]", "core"),
    Feature("SF003", "E-Stop Handler", "Safety", "pattern", r"estop|e_stop|EStop", "core"),
    Feature("SF004", "Soft Limits", "Safety", "pattern", r"soft.?limit|SoftLimit", "core"),
    Feature("SF005", "Hard Limits", "Safety", "pattern", r"hard.?limit|HardLimit", "core"),
    Feature("SF006", "Deadman Switch", "Safety", "pattern", r"deadman|dead.?man", "core"),
    Feature("SF007", "Velocity Monitoring", "Safety", "pattern", r"velocity.?monitor|VelocityMonitor", "core"),

    # ============= KINEMATICS =============
    Feature("KN001", "ForwardKinematics", "Kinematics", "file", "kinematics/ForwardKinematics.hpp", "core"),
    Feature("KN002", "InverseKinematics", "Kinematics", "file", "kinematics/InverseKinematics.hpp", "core"),
    Feature("KN003", "DH Transform", "Kinematics", "pattern", r"DH|dhTransform|dh_transform", "core"),
    Feature("KN004", "Jacobian Computation", "Kinematics", "pattern", r"jacobian|Jacobian", "core"),
    Feature("KN005", "8 IK Configurations", "Kinematics", "pattern", r"shoulder|elbow|wrist|config", "core"),
    Feature("KN006", "Singularity Detection", "Kinematics", "pattern", r"singularity|Singularity", "core"),

    # ============= TRAJECTORY =============
    Feature("TJ001", "TrajectoryPlanner", "Trajectory", "file", "trajectory/TrajectoryPlanner.hpp", "core"),
    Feature("TJ002", "TrajectoryExecutor", "Trajectory", "file", "trajectory/TrajectoryExecutor.hpp", "core"),
    Feature("TJ003", "Ruckig OTG Integration", "Trajectory", "pattern", r"ruckig|Ruckig", "core"),
    Feature("TJ004", "S-Curve Profile", "Trajectory", "pattern", r"s.?curve|scurve|jerk.?limit", "core"),
    Feature("TJ005", "PTP Motion", "Trajectory", "pattern", r"PTP|point.?to.?point", "core"),
    Feature("TJ006", "Linear Motion (MOVL)", "Trajectory", "pattern", r"MOVL|linear.?motion|LinearMove", "core"),
    Feature("TJ007", "Circular Motion (MOVC)", "Trajectory", "pattern", r"MOVC|circular|arc.?motion", "core"),
    Feature("TJ008", "Jog Mode", "Trajectory", "pattern", r"jog|Jog", "core"),

    # ============= MOTION CONTROLLER =============
    Feature("MC001", "RobotController", "Motion Controller", "file", "controller/RobotController.cpp", "core"),
    Feature("MC002", "Motion Loop (1kHz)", "Motion Controller", "pattern", r"1000.*Hz|1kHz|motion.?loop", "core"),
    Feature("MC003", "Joint Jog", "Motion Controller", "pattern", r"joint.?jog|JointJog", "core"),
    Feature("MC004", "Cartesian Jog", "Motion Controller", "pattern", r"cartesian.?jog|CartesianJog", "core"),
    Feature("MC005", "Speed Override", "Motion Controller", "pattern", r"speed.?override|SpeedOverride|speed.?ratio", "core"),

    # ============= FIRMWARE =============
    Feature("FW001", "FirmwareInterface", "Firmware", "file", "firmware/FirmwareInterface.hpp", "core"),
    Feature("FW002", "SerialPort", "Firmware", "file", "firmware/SerialPort.hpp", "core"),
    Feature("FW003", "G-code Generation", "Firmware", "pattern", r"G1|gcode|g.?code", "core"),
    Feature("FW004", "grblHAL Protocol", "Firmware", "pattern", r"grbl|GRBL|grblHAL", "core"),
    Feature("FW005", "Status Parsing", "Firmware", "pattern", r"parseStatus|parse.?status|status.?report", "core"),
    Feature("FW006", "MotionStreamer", "Firmware", "file", "firmware/MotionStreamer.hpp", "core"),

    # ============= WELDING =============
    Feature("WD001", "WeldingStateMachine", "Welding", "file", "welding/WeldingStateMachine.hpp", "core"),
    Feature("WD002", "WeldingController", "Welding", "file", "welding/WeldingController.hpp", "core"),
    Feature("WD003", "WeldingIO", "Welding", "file", "welding/WeldingIO.hpp", "core"),
    Feature("WD004", "State: PREFLOW", "Welding", "pattern", r"PREFLOW|PreFlow|pre.?flow", "core"),
    Feature("WD005", "State: IGNITION", "Welding", "pattern", r"IGNITION|Ignition", "core"),
    Feature("WD006", "State: WELD", "Welding", "pattern", r"State::WELD|WELDING", "core"),
    Feature("WD007", "State: CRATER", "Welding", "pattern", r"CRATER|Crater", "core"),
    Feature("WD008", "State: BURNBACK", "Welding", "pattern", r"BURNBACK|BurnBack|burn.?back", "core"),
    Feature("WD009", "State: POSTFLOW", "Welding", "pattern", r"POSTFLOW|PostFlow|post.?flow", "core"),
    Feature("WD010", "Arc Monitor", "Welding", "pattern", r"arc.?monitor|ArcMonitor", "core"),
    Feature("WD011", "Fault Detection", "Welding", "pattern", r"fault|Fault|IGNITION_FAILURE|ARC_LOST", "core"),

    # ============= WEAVING =============
    Feature("WV001", "WeavePatternGenerator", "Weaving", "file", "welding/WeavePatternGenerator.hpp", "core"),
    Feature("WV002", "WeaveExecutor", "Weaving", "file", "welding/WeaveExecutor.hpp", "core"),
    Feature("WV003", "Pattern: Sine", "Weaving", "pattern", r"sine|Sine|SINE", "core"),
    Feature("WV004", "Pattern: Triangle", "Weaving", "pattern", r"triangle|Triangle|TRIANGLE", "core"),
    Feature("WV005", "Pattern: Trapezoid", "Weaving", "pattern", r"trapezoid|Trapezoid|TRAPEZOID", "core"),
    Feature("WV006", "Pattern: Circle", "Weaving", "pattern", r"circle|Circle|CIRCLE", "core"),
    Feature("WV007", "Pattern: Figure-8", "Weaving", "pattern", r"figure.?8|lissajous|Lissajous", "core"),
    Feature("WV008", "Dwell Time", "Weaving", "pattern", r"dwell|Dwell", "core"),

    # ============= VISION =============
    Feature("VS001", "HikrobotLaserProfiler", "Vision", "file", "vision/HikrobotLaserProfiler.cpp", "core"),
    Feature("VS002", "ProfileProcessor", "Vision", "file", "vision/ProfileProcessor.cpp", "core"),
    Feature("VS003", "SeamTracker", "Vision", "file", "vision/SeamTracker.cpp", "core"),
    Feature("VS004", "JointDetector", "Vision", "file", "vision/JointDetector.cpp", "core"),
    Feature("VS005", "SensorManager", "Vision", "file", "vision/SensorManager.cpp", "core"),
    Feature("VS006", "V-Groove Detection", "Vision", "pattern", r"v.?groove|VGroove|groove", "core"),
    Feature("VS007", "Fillet Detection", "Vision", "pattern", r"fillet|Fillet", "core"),
    Feature("VS008", "RANSAC", "Vision", "pattern", r"ransac|RANSAC", "core"),
    Feature("VS009", "Kalman Filter", "Vision", "pattern", r"kalman|Kalman", "core"),
    Feature("VS010", "Latency Compensation", "Vision", "pattern", r"latency|Latency", "core"),
    Feature("VS011", "Point Cloud Processing", "Vision", "pattern", r"point.?cloud|PointCloud", "core"),
    Feature("VS012", "Noise Filtering (SOR/ROR)", "Vision", "pattern", r"SOR|ROR|outlier.?removal", "core"),

    # ============= HMI CORE =============
    Feature("HM001", "MainViewModel", "HMI", "file", "RobotController.UI/ViewModels/MainViewModel.cs", "ui"),
    Feature("HM002", "IpcClientService", "HMI", "file", "RobotController.Common/Services/IpcClientService.cs", "ui"),
    Feature("HM003", "JointPositionViewModel", "HMI", "file", "RobotController.UI/ViewModels/JointPositionViewModel.cs", "ui"),
    Feature("HM004", "CartesianPositionViewModel", "HMI", "file", "RobotController.UI/ViewModels/CartesianPositionViewModel.cs", "ui"),
    Feature("HM005", "MotionControlViewModel", "HMI", "file", "RobotController.UI/ViewModels/MotionControlViewModel.cs", "ui"),

    # ============= HMI VIEWS =============
    Feature("HV001", "MainWindow", "HMI Views", "file", "RobotController.UI/Views/MainWindow.xaml.cs", "ui"),
    Feature("HV002", "JogPanel", "HMI Views", "file", "RobotController.UI/Views/Controls/JogPanel.xaml.cs", "ui"),
    Feature("HV003", "MotionControlPanel", "HMI Views", "file", "RobotController.UI/Views/Controls/MotionControlPanel.xaml.cs", "ui"),
    Feature("HV004", "PositionDisplay", "HMI Views", "file", "RobotController.UI/Views/Controls/PositionDisplay.xaml.cs", "ui"),

    # ============= HMI 3D VIEWPORT =============
    Feature("H3D001", "RobotModel3D", "HMI 3D", "file", "RobotController.UI/Models/RobotModel3D.cs", "ui"),
    Feature("H3D002", "ViewportService", "HMI 3D", "file", "RobotController.UI/Services/ViewportService.cs", "ui"),
    Feature("H3D003", "Helix Toolkit Integration", "HMI 3D", "pattern", r"HelixToolkit|Helix", "ui"),
    Feature("H3D004", "STL Loader", "HMI 3D", "pattern", r"StlReader|\.stl|STL", "ui"),

    # ============= HMI WELDING =============
    Feature("HW001", "WeldingControlViewModel", "HMI Welding", "file", "RobotController.UI/ViewModels/Welding/WeldingControlViewModel.cs", "ui"),
    Feature("HW002", "WeldingStateViewModel", "HMI Welding", "file", "RobotController.UI/ViewModels/Welding/WeldingStateViewModel.cs", "ui"),
    Feature("HW003", "WeavePreviewViewModel", "HMI Welding", "file", "RobotController.UI/ViewModels/Welding/WeavePreviewViewModel.cs", "ui"),
    Feature("HW004", "WeldingControlPanel", "HMI Welding", "file", "RobotController.UI/Views/Controls/Welding/WeldingControlPanel.xaml.cs", "ui"),
    Feature("HW005", "WeavePreviewPanel", "HMI Welding", "file", "RobotController.UI/Views/Controls/Welding/WeavePreviewPanel.xaml.cs", "ui"),
    Feature("HW006", "WeldingFeedbackPanel", "HMI Welding", "file", "RobotController.UI/Views/Controls/Welding/WeldingFeedbackPanel.xaml.cs", "ui"),

    # ============= HMI VISION =============
    Feature("HVS001", "VisionControlViewModel", "HMI Vision", "file", "RobotController.UI/ViewModels/Vision/VisionControlViewModel.cs", "ui"),
    Feature("HVS002", "ProfileDisplayViewModel", "HMI Vision", "file", "RobotController.UI/ViewModels/Vision/ProfileDisplayViewModel.cs", "ui"),
    Feature("HVS003", "SensorStatusViewModel", "HMI Vision", "file", "RobotController.UI/ViewModels/Vision/SensorStatusViewModel.cs", "ui"),
    Feature("HVS004", "TrackingStatusViewModel", "HMI Vision", "file", "RobotController.UI/ViewModels/Vision/TrackingStatusViewModel.cs", "ui"),
    Feature("HVS005", "CalibrationViewModel", "HMI Vision", "file", "RobotController.UI/ViewModels/Vision/CalibrationViewModel.cs", "ui"),
    Feature("HVS006", "ProfileDisplayControl", "HMI Vision", "file", "RobotController.UI/Views/Controls/Vision/ProfileDisplayControl.xaml.cs", "ui"),
    Feature("HVS007", "VisionMainPanel", "HMI Vision", "file", "RobotController.UI/Views/Controls/Vision/VisionMainPanel.xaml.cs", "ui"),

    # ============= TESTS =============
    Feature("TS001", "test_config", "Tests", "file", "tests/test_config.cpp", "core"),
    Feature("TS002", "test_ipc", "Tests", "file", "tests/test_ipc.cpp", "core"),
    Feature("TS003", "test_kinematics", "Tests", "file", "tests/test_kinematics.cpp", "core"),
    Feature("TS004", "test_state_machine", "Tests", "file", "tests/test_state_machine.cpp", "core"),
    Feature("TS005", "test_trajectory", "Tests", "file", "tests/test_trajectory.cpp", "core"),
    Feature("TS006", "test_welding", "Tests", "file", "tests/test_welding.cpp", "core"),
    Feature("TS007", "test_weaving", "Tests", "file", "tests/test_weaving.cpp", "core"),
    Feature("TS008", "test_vision", "Tests", "file", "tests/test_vision.cpp", "core"),
    Feature("TS009", "test_seam", "Tests", "file", "tests/test_seam.cpp", "core"),
    Feature("TS010", "test_firmware", "Tests", "file", "tests/test_firmware.cpp", "core"),
]


def get_base_path(layer: str) -> Path:
    """Get base path for a layer."""
    if layer == "core":
        return CORE_SRC
    elif layer == "ui":
        return UI_SRC
    else:
        return PROJECT_ROOT


def count_lines(file_path: Path) -> int:
    """Count non-empty, non-comment lines in a file."""
    if not file_path.exists():
        return 0

    count = 0
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            in_block_comment = False
            for line in f:
                stripped = line.strip()

                # Skip empty lines
                if not stripped:
                    continue

                # Handle block comments
                if '/*' in stripped:
                    in_block_comment = True
                if '*/' in stripped:
                    in_block_comment = False
                    continue
                if in_block_comment:
                    continue

                # Skip single-line comments
                if stripped.startswith('//') or stripped.startswith('#'):
                    continue

                count += 1
    except Exception:
        return 0

    return count


def is_stub_only(file_path: Path) -> bool:
    """Check if a file is likely just a stub (minimal implementation)."""
    loc = count_lines(file_path)

    # Very short files are likely stubs
    if loc < 10:
        return True

    # Check for TODO/NotImplemented patterns
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            stub_patterns = [
                r'TODO.*implement',
                r'NotImplemented',
                r'throw.*not.*implement',
                r'// stub',
                r'// placeholder',
            ]
            for pattern in stub_patterns:
                if re.search(pattern, content, re.IGNORECASE):
                    return True
    except Exception:
        pass

    return False


def search_pattern_in_files(base_path: Path, pattern: str, extensions: List[str]) -> tuple:
    """Search for a pattern in files, return (found, file, match_count)."""
    if not base_path.exists():
        return False, None, 0

    try:
        regex = re.compile(pattern, re.IGNORECASE)
    except re.error:
        return False, None, 0

    total_matches = 0
    found_file = None

    for ext in extensions:
        for file_path in base_path.rglob(f"*{ext}"):
            if "/build/" in str(file_path) or "/obj/" in str(file_path):
                continue
            try:
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    matches = regex.findall(content)
                    if matches:
                        total_matches += len(matches)
                        if found_file is None:
                            found_file = file_path
            except Exception:
                continue

    return total_matches > 0, found_file, total_matches


def verify_feature(feature: Feature) -> Feature:
    """Verify a single feature and update its status."""
    base_path = get_base_path(feature.layer)

    if feature.layer == "core":
        extensions = [".cpp", ".hpp", ".h"]
    else:
        extensions = [".cs"]

    if feature.check_type == "file":
        # Check if file exists
        if feature.layer == "core" and feature.check_target.startswith("tests/"):
            file_path = CORE_TESTS / feature.check_target.replace("tests/", "")
        else:
            file_path = base_path / feature.check_target

        if file_path.exists():
            feature.loc = count_lines(file_path)
            if is_stub_only(file_path):
                feature.status = Status.STUB
                feature.details = f"File exists but appears to be stub ({feature.loc} LOC)"
            elif feature.loc < 30:
                feature.status = Status.PARTIAL
                feature.details = f"File exists with minimal code ({feature.loc} LOC)"
            else:
                feature.status = Status.IMPLEMENTED
                feature.details = f"{feature.loc} LOC"
        else:
            feature.status = Status.MISSING
            feature.details = f"File not found: {file_path}"

    elif feature.check_type == "class":
        # Search for class definition
        class_pattern = rf"class\s+{feature.check_target}\b|struct\s+{feature.check_target}\b"
        found, found_file, match_count = search_pattern_in_files(base_path, class_pattern, extensions)

        if found:
            feature.status = Status.IMPLEMENTED
            feature.details = f"Found in {found_file.name}"
        else:
            feature.status = Status.MISSING
            feature.details = f"Class '{feature.check_target}' not found"

    elif feature.check_type == "function":
        # Search for function definition
        func_pattern = rf"\b{feature.check_target}\s*\("
        found, found_file, match_count = search_pattern_in_files(base_path, func_pattern, extensions)

        if found:
            feature.status = Status.IMPLEMENTED
            feature.details = f"Found {match_count} references"
        else:
            feature.status = Status.MISSING
            feature.details = f"Function '{feature.check_target}' not found"

    elif feature.check_type == "pattern":
        # Search for pattern
        found, found_file, match_count = search_pattern_in_files(base_path, feature.check_target, extensions)

        if found:
            if match_count >= 3:
                feature.status = Status.IMPLEMENTED
            else:
                feature.status = Status.PARTIAL
            feature.details = f"Found {match_count} matches"
        else:
            feature.status = Status.MISSING
            feature.details = "Pattern not found in codebase"

    return feature


def generate_report(features: List[Feature], output_format: str = "markdown") -> str:
    """Generate verification report."""
    # Group by category
    categories: Dict[str, List[Feature]] = {}
    for f in features:
        if f.category not in categories:
            categories[f.category] = []
        categories[f.category].append(f)

    # Calculate stats
    total = len(features)
    implemented = sum(1 for f in features if f.status == Status.IMPLEMENTED)
    partial = sum(1 for f in features if f.status == Status.PARTIAL)
    stub = sum(1 for f in features if f.status == Status.STUB)
    missing = sum(1 for f in features if f.status == Status.MISSING)

    if output_format == "json":
        return json.dumps({
            "summary": {
                "total": total,
                "implemented": implemented,
                "partial": partial,
                "stub": stub,
                "missing": missing,
                "completion_percent": round(implemented / total * 100, 1) if total > 0 else 0
            },
            "features": [
                {
                    "id": f.id,
                    "name": f.name,
                    "category": f.category,
                    "status": f.status.name,
                    "details": f.details,
                    "loc": f.loc
                }
                for f in features
            ]
        }, indent=2)

    # Markdown report
    lines = [
        "# Feature Verification Report",
        "",
        f"**Generated:** {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "## Summary",
        "",
        "| Status | Count | Percent |",
        "|--------|-------|---------|",
        f"| ✅ Implemented | {implemented} | {implemented/total*100:.1f}% |",
        f"| 🔶 Partial | {partial} | {partial/total*100:.1f}% |",
        f"| ⚠️ Stub Only | {stub} | {stub/total*100:.1f}% |",
        f"| ❌ Missing | {missing} | {missing/total*100:.1f}% |",
        f"| **Total** | **{total}** | **100%** |",
        "",
        "---",
        "",
    ]

    for category, feats in categories.items():
        cat_impl = sum(1 for f in feats if f.status == Status.IMPLEMENTED)
        cat_total = len(feats)

        lines.append(f"## {category} ({cat_impl}/{cat_total})")
        lines.append("")
        lines.append("| ID | Feature | Status | Details |")
        lines.append("|----|---------|--------|---------|")

        for f in feats:
            lines.append(f"| {f.id} | {f.name} | {f.status.value} | {f.details} |")

        lines.append("")

    # Missing features summary
    missing_features = [f for f in features if f.status == Status.MISSING]
    if missing_features:
        lines.append("---")
        lines.append("")
        lines.append("## Missing Features (Action Required)")
        lines.append("")
        for f in missing_features:
            lines.append(f"- **{f.id}**: {f.name} ({f.category})")
        lines.append("")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Verify features in Robot Controller project")
    parser.add_argument("--output", "-o", help="Output file path", default=None)
    parser.add_argument("--json", action="store_true", help="Output as JSON")
    parser.add_argument("--category", "-c", help="Filter by category")
    args = parser.parse_args()

    print("Verifying features...")

    features = FEATURE_REGISTRY.copy()

    # Filter by category if specified
    if args.category:
        features = [f for f in features if args.category.lower() in f.category.lower()]

    # Verify each feature
    for i, feature in enumerate(features):
        features[i] = verify_feature(feature)
        status_char = "+" if features[i].status == Status.IMPLEMENTED else "-" if features[i].status == Status.MISSING else "?"
        print(f"  [{status_char}] {feature.id}: {feature.name}")

    # Generate report
    output_format = "json" if args.json else "markdown"
    report = generate_report(features, output_format)

    if args.output:
        output_path = Path(args.output)
        output_path.write_text(report, encoding='utf-8')
        print(f"\nReport saved to: {output_path}")
    else:
        print("\n" + "=" * 60)
        print(report)


if __name__ == "__main__":
    main()
