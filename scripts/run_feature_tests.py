#!/usr/bin/env python3
"""
Run Feature Tests and Generate Report

This script runs all feature tests and generates a comprehensive report
showing which features are tested and their pass/fail status.

Usage:
    python run_feature_tests.py [--build] [--filter PATTERN] [--output FILE]

Options:
    --build         Rebuild tests before running
    --filter        Filter tests by pattern (e.g., "SF*" for safety tests)
    --output        Output file for report (default: FEATURE_TEST_REPORT.md)
    --xml           Generate JUnit XML output
"""

import subprocess
import os
import sys
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import re

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
CORE_BUILD = PROJECT_ROOT / "src" / "core" / "build"
DOCS_DIR = PROJECT_ROOT / "docs"

@dataclass
class TestResult:
    name: str
    status: str  # "PASSED", "FAILED", "SKIPPED"
    duration_ms: float = 0
    failure_message: str = ""
    feature_id: str = ""

@dataclass
class TestSuite:
    name: str
    tests: List[TestResult] = field(default_factory=list)
    passed: int = 0
    failed: int = 0
    skipped: int = 0

# Feature ID to test mapping
FEATURE_TEST_MAP = {
    # Core Platform
    "CP001": ["*CP001*", "*ConfigManager*"],
    "CP002": ["*CP002*", "*RobotConfig*"],
    "CP003": ["*CP003*", "*SystemConfig*"],
    "CP004": ["*CP004*", "*IpcServer*"],
    "CP005": ["*CP005*", "*ReqRep*"],
    "CP006": ["*CP006*", "*PubSub*"],
    "CP007": ["*CP007*", "*Message*Serialization*"],
    "CP008": ["*CP008*", "*CRC32*"],
    "CP009": ["*CP009*", "*Heartbeat*"],
    "CP010": ["*CP010*", "*Logger*"],

    # State Machine
    "SM001": ["*StateMachine*Initial*", "*StateMachine*"],
    "SM002": ["*Initial*", "*BOOT*"],
    "SM003": ["*EStop*"],
    "SM004": ["*Idle*"],
    "SM005": ["*Ready*", "*Moving*", "*Operational*"],
    "SM006": ["*T1*", "*Mode*"],
    "SM007": ["*T2*", "*Mode*"],
    "SM008": ["*AUTO*", "*Mode*"],

    # Safety
    "SF001": ["*SF001*", "*SafetyMonitor*"],
    "SF002": ["*SF002*", "*DualChannel*"],
    "SF003": ["*SF003*", "*EStop*"],
    "SF004": ["*SF004*", "*SoftLimit*"],
    "SF005": ["*SF005*", "*HardLimit*"],
    "SF006": ["*SF006*", "*Deadman*"],
    "SF007": ["*SF007*", "*Velocity*"],

    # Kinematics
    "KN001": ["*FK_*", "*ForwardKinematics*"],
    "KN002": ["*IK_*", "*InverseKinematics*"],
    "KN003": ["*DH*"],
    "KN004": ["*Jacobian*"],
    "KN005": ["*MultiSolution*", "*8*Config*"],
    "KN006": ["*Singularity*"],

    # Trajectory
    "TJ001": ["*Planner*"],
    "TJ002": ["*Executor*"],
    "TJ003": ["*Ruckig*", "*OTG*"],
    "TJ004": ["*SCurve*", "*Jerk*"],
    "TJ005": ["*PTP*"],
    "TJ006": ["*Linear*", "*MOVL*"],
    "TJ007": ["*Circular*", "*Arc*"],
    "TJ008": ["*Jog*"],

    # Motion Controller
    "MC001": ["*MC001*", "*RobotController*"],
    "MC002": ["*MC002*", "*MotionLoop*", "*1kHz*"],
    "MC003": ["*MC003*", "*JointJog*"],
    "MC004": ["*MC004*", "*CartesianJog*"],
    "MC005": ["*MC005*", "*SpeedOverride*"],

    # Firmware
    "FW001": ["*Interface*"],
    "FW002": ["*Serial*"],
    "FW003": ["*Gcode*", "*Command*"],
    "FW004": ["*Grbl*", "*Protocol*"],
    "FW005": ["*Status*Parse*"],
    "FW006": ["*Streamer*"],

    # Welding
    "WD001": ["*WeldingStateMachine*"],
    "WD002": ["*WeldingController*"],
    "WD003": ["*WeldingIO*", "*IO*"],
    "WD004": ["*Preflow*"],
    "WD005": ["*Ignition*"],
    "WD006": ["*Weld*"],
    "WD007": ["*Crater*"],
    "WD008": ["*Burnback*"],
    "WD009": ["*Postflow*"],
    "WD010": ["*ArcMonitor*"],
    "WD011": ["*Fault*"],

    # Weaving
    "WV001": ["*PatternGenerator*"],
    "WV002": ["*WeaveExecutor*", "*Executor*"],
    "WV003": ["*Sine*"],
    "WV004": ["*Triangle*"],
    "WV005": ["*Trapezoid*"],
    "WV006": ["*Circle*"],
    "WV007": ["*Figure8*", "*Lissajous*"],
    "WV008": ["*Dwell*"],

    # Vision
    "VS001": ["*LaserProfiler*", "*Hikrobot*"],
    "VS002": ["*ProfileProcessor*"],
    "VS003": ["*SeamTracker*"],
    "VS004": ["*JointDetector*"],
    "VS005": ["*SensorManager*"],
    "VS006": ["*VGroove*", "*Groove*"],
    "VS007": ["*Fillet*"],
    "VS008": ["*RANSAC*"],
    "VS009": ["*Kalman*"],
    "VS010": ["*Latency*"],
    "VS011": ["*PointCloud*"],
    "VS012": ["*Noise*", "*Filter*"],
}

# Test executables
TEST_EXECUTABLES = [
    "test_core_platform",
    "test_state_machine",
    "test_safety",
    "test_kinematics",
    "test_trajectory",
    "test_motion_controller",
    "test_firmware",
    "test_welding",
    "test_weaving",
    "test_vision",
    "test_seam",
    "test_feature_registry",
]


def find_test_executables(build_dir: Path) -> List[Path]:
    """Find all test executables in build directory."""
    executables = []
    bin_dir = build_dir / "bin"

    if not bin_dir.exists():
        bin_dir = build_dir

    for name in TEST_EXECUTABLES:
        for ext in ["", ".exe"]:
            path = bin_dir / f"{name}{ext}"
            if path.exists():
                executables.append(path)
                break

    return executables


def run_tests(executable: Path, filter_pattern: str = "",
              xml_output: Optional[Path] = None) -> Tuple[int, str, str]:
    """Run a test executable and return results."""
    cmd = [str(executable)]

    if filter_pattern:
        cmd.append(f"--gtest_filter={filter_pattern}")

    if xml_output:
        cmd.append(f"--gtest_output=xml:{xml_output}")

    cmd.append("--gtest_color=yes")

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "Test timed out"
    except Exception as e:
        return -1, "", str(e)


def parse_gtest_output(output: str) -> List[TestResult]:
    """Parse Google Test output to extract test results."""
    results = []

    # Pattern for test results
    run_pattern = r"\[ RUN      \] (\S+)"
    ok_pattern = r"\[       OK \] (\S+) \((\d+) ms\)"
    fail_pattern = r"\[  FAILED  \] (\S+)"

    current_test = None

    for line in output.split("\n"):
        run_match = re.search(run_pattern, line)
        if run_match:
            current_test = run_match.group(1)
            continue

        ok_match = re.search(ok_pattern, line)
        if ok_match:
            name = ok_match.group(1)
            duration = float(ok_match.group(2))

            # Extract feature ID from test name
            feature_id = ""
            for fid in FEATURE_TEST_MAP.keys():
                if fid in name:
                    feature_id = fid
                    break

            results.append(TestResult(
                name=name,
                status="PASSED",
                duration_ms=duration,
                feature_id=feature_id
            ))
            continue

        fail_match = re.search(fail_pattern, line)
        if fail_match:
            name = fail_match.group(1)
            feature_id = ""
            for fid in FEATURE_TEST_MAP.keys():
                if fid in name:
                    feature_id = fid
                    break

            results.append(TestResult(
                name=name,
                status="FAILED",
                feature_id=feature_id
            ))

    return results


def parse_xml_results(xml_path: Path) -> List[TestResult]:
    """Parse JUnit XML output from Google Test."""
    if not xml_path.exists():
        return []

    results = []
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for testsuite in root.findall(".//testsuite"):
            for testcase in testsuite.findall("testcase"):
                name = f"{testcase.get('classname')}.{testcase.get('name')}"
                duration = float(testcase.get('time', 0)) * 1000

                failure = testcase.find("failure")
                if failure is not None:
                    status = "FAILED"
                    message = failure.text or ""
                else:
                    status = "PASSED"
                    message = ""

                # Extract feature ID
                feature_id = ""
                for fid in FEATURE_TEST_MAP.keys():
                    if fid in name:
                        feature_id = fid
                        break

                results.append(TestResult(
                    name=name,
                    status=status,
                    duration_ms=duration,
                    failure_message=message,
                    feature_id=feature_id
                ))

    except Exception as e:
        print(f"Error parsing XML: {e}")

    return results


def map_tests_to_features(results: List[TestResult]) -> Dict[str, List[TestResult]]:
    """Map test results to feature IDs."""
    feature_results: Dict[str, List[TestResult]] = {fid: [] for fid in FEATURE_TEST_MAP}

    for result in results:
        # First check if feature ID is already set
        if result.feature_id and result.feature_id in feature_results:
            feature_results[result.feature_id].append(result)
            continue

        # Otherwise, try to match by pattern
        for feature_id, patterns in FEATURE_TEST_MAP.items():
            for pattern in patterns:
                # Convert glob pattern to regex
                regex = pattern.replace("*", ".*")
                if re.search(regex, result.name, re.IGNORECASE):
                    feature_results[feature_id].append(result)
                    result.feature_id = feature_id
                    break

    return feature_results


def generate_report(feature_results: Dict[str, List[TestResult]],
                    output_path: Path) -> None:
    """Generate markdown report."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Calculate statistics
    total_features = len(FEATURE_TEST_MAP)
    tested_features = sum(1 for tests in feature_results.values() if tests)
    passed_features = sum(1 for tests in feature_results.values()
                          if tests and all(t.status == "PASSED" for t in tests))
    failed_features = sum(1 for tests in feature_results.values()
                          if tests and any(t.status == "FAILED" for t in tests))

    total_tests = sum(len(tests) for tests in feature_results.values())
    passed_tests = sum(1 for tests in feature_results.values()
                       for t in tests if t.status == "PASSED")
    failed_tests = sum(1 for tests in feature_results.values()
                       for t in tests if t.status == "FAILED")

    lines = [
        "# Feature Test Report",
        "",
        f"**Generated:** {now}",
        "",
        "## Summary",
        "",
        "| Metric | Value |",
        "|--------|-------|",
        f"| Total Features | {total_features} |",
        f"| Features with Tests | {tested_features} ({100*tested_features//total_features}%) |",
        f"| Features Passing | {passed_features} |",
        f"| Features Failing | {failed_features} |",
        f"| Total Tests | {total_tests} |",
        f"| Tests Passed | {passed_tests} |",
        f"| Tests Failed | {failed_tests} |",
        "",
        "---",
        "",
    ]

    # Group features by category
    categories = {
        "Core Platform": ["CP001", "CP002", "CP003", "CP004", "CP005", "CP006", "CP007", "CP008", "CP009", "CP010"],
        "State Machine": ["SM001", "SM002", "SM003", "SM004", "SM005", "SM006", "SM007", "SM008"],
        "Safety": ["SF001", "SF002", "SF003", "SF004", "SF005", "SF006", "SF007"],
        "Kinematics": ["KN001", "KN002", "KN003", "KN004", "KN005", "KN006"],
        "Trajectory": ["TJ001", "TJ002", "TJ003", "TJ004", "TJ005", "TJ006", "TJ007", "TJ008"],
        "Motion Controller": ["MC001", "MC002", "MC003", "MC004", "MC005"],
        "Firmware": ["FW001", "FW002", "FW003", "FW004", "FW005", "FW006"],
        "Welding": ["WD001", "WD002", "WD003", "WD004", "WD005", "WD006", "WD007", "WD008", "WD009", "WD010", "WD011"],
        "Weaving": ["WV001", "WV002", "WV003", "WV004", "WV005", "WV006", "WV007", "WV008"],
        "Vision": ["VS001", "VS002", "VS003", "VS004", "VS005", "VS006", "VS007", "VS008", "VS009", "VS010", "VS011", "VS012"],
    }

    for category, feature_ids in categories.items():
        cat_tested = sum(1 for fid in feature_ids if feature_results.get(fid))
        cat_passed = sum(1 for fid in feature_ids
                         if feature_results.get(fid) and
                         all(t.status == "PASSED" for t in feature_results[fid]))

        lines.append(f"## {category} ({cat_passed}/{len(feature_ids)} passing)")
        lines.append("")
        lines.append("| ID | Status | Tests | Details |")
        lines.append("|----|--------|-------|---------|")

        for fid in feature_ids:
            tests = feature_results.get(fid, [])
            if not tests:
                status = "⚪ No Tests"
                test_count = "0"
                details = "No tests defined"
            elif all(t.status == "PASSED" for t in tests):
                status = "✅ Pass"
                test_count = str(len(tests))
                details = f"{sum(t.duration_ms for t in tests):.0f}ms total"
            else:
                status = "❌ Fail"
                test_count = f"{sum(1 for t in tests if t.status == 'PASSED')}/{len(tests)}"
                failed = [t.name for t in tests if t.status == "FAILED"]
                details = f"Failed: {', '.join(failed[:3])}"

            lines.append(f"| {fid} | {status} | {test_count} | {details} |")

        lines.append("")

    # Failed tests section
    failed_tests_list = [t for tests in feature_results.values()
                         for t in tests if t.status == "FAILED"]
    if failed_tests_list:
        lines.append("---")
        lines.append("")
        lines.append("## Failed Tests")
        lines.append("")
        for test in failed_tests_list:
            lines.append(f"- **{test.name}**")
            if test.failure_message:
                lines.append(f"  ```")
                lines.append(f"  {test.failure_message[:200]}")
                lines.append(f"  ```")
        lines.append("")

    # Untested features section
    untested = [fid for fid, tests in feature_results.items() if not tests]
    if untested:
        lines.append("---")
        lines.append("")
        lines.append("## Untested Features")
        lines.append("")
        for fid in untested:
            lines.append(f"- {fid}")
        lines.append("")

    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Report saved to: {output_path}")


def build_tests(build_dir: Path) -> bool:
    """Build test executables."""
    print("Building tests...")

    if not build_dir.exists():
        build_dir.mkdir(parents=True)

    # Configure
    result = subprocess.run(
        ["cmake", ".."],
        cwd=build_dir,
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        print(f"CMake configure failed: {result.stderr}")
        return False

    # Build
    result = subprocess.run(
        ["cmake", "--build", ".", "--config", "Release"],
        cwd=build_dir,
        capture_output=True,
        text=True
    )
    if result.returncode != 0:
        print(f"Build failed: {result.stderr}")
        return False

    print("Build successful!")
    return True


def main():
    parser = argparse.ArgumentParser(description="Run feature tests")
    parser.add_argument("--build", action="store_true", help="Build before running")
    parser.add_argument("--filter", default="", help="Test filter pattern")
    parser.add_argument("--output", default="FEATURE_TEST_REPORT.md", help="Output file")
    parser.add_argument("--xml", action="store_true", help="Generate XML output")
    args = parser.parse_args()

    # Build if requested
    if args.build:
        if not build_tests(CORE_BUILD):
            sys.exit(1)

    # Find executables
    executables = find_test_executables(CORE_BUILD)
    if not executables:
        print(f"No test executables found in {CORE_BUILD}")
        print("Run with --build to build tests first")
        sys.exit(1)

    print(f"Found {len(executables)} test executables")

    # Run all tests
    all_results: List[TestResult] = []

    for exe in executables:
        print(f"\nRunning: {exe.name}")

        xml_output = None
        if args.xml:
            xml_output = CORE_BUILD / f"{exe.stem}_results.xml"

        returncode, stdout, stderr = run_tests(exe, args.filter, xml_output)

        if returncode == -1:
            print(f"  Error: {stderr}")
            continue

        # Parse results
        if xml_output and xml_output.exists():
            results = parse_xml_results(xml_output)
        else:
            results = parse_gtest_output(stdout)

        passed = sum(1 for r in results if r.status == "PASSED")
        failed = sum(1 for r in results if r.status == "FAILED")
        print(f"  {passed} passed, {failed} failed")

        all_results.extend(results)

    # Map to features
    feature_results = map_tests_to_features(all_results)

    # Generate report
    output_path = DOCS_DIR / args.output
    generate_report(feature_results, output_path)

    # Print summary
    print("\n" + "=" * 60)
    total = len(all_results)
    passed = sum(1 for r in all_results if r.status == "PASSED")
    failed = sum(1 for r in all_results if r.status == "FAILED")
    print(f"Total: {total} tests, {passed} passed, {failed} failed")

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
