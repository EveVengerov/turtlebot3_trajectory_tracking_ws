#!/usr/bin/env python3

"""
Test runner script for turtlebot3_tracker package.
This script provides a convenient way to run different types of tests.
"""

import os
import sys
import subprocess
import argparse
import time
from pathlib import Path


def run_command(command, description):
    """Run a command and return the result."""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Command: {' '.join(command)}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        print("✅ SUCCESS")
        if result.stdout:
            print("Output:")
            print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print("❌ FAILED")
        print(f"Return code: {e.returncode}")
        if e.stdout:
            print("Stdout:")
            print(e.stdout)
        if e.stderr:
            print("Stderr:")
            print(e.stderr)
        return False


def run_unit_tests():
    """Run unit tests."""
    print("\n🧪 Running Unit Tests")
    
    # Run component tests (no ROS dependencies)
    component_tests = [
        "python3 -m pytest test_trajectory_components.py -v"
    ]
    
    for test in component_tests:
        success = run_command(test.split(), f"Component test: {test}")
        if not success:
            return False
    
    return True


def run_integration_tests():
    """Run integration tests."""
    print("\n🔗 Running Integration Tests")
    
    # Run trajectory tracker tests
    trajectory_tests = [
        "python3 -m pytest test_trajectory_tracker.py -v"
    ]
    
    for test in trajectory_tests:
        success = run_command(test.split(), f"Trajectory test: {test}")
        if not success:
            return False
    
    return True


def run_launch_tests():
    """Run launch file tests."""
    print("\n🚀 Running Launch Tests")
    
    # Run launch file tests
    launch_tests = [
        "python3 -m pytest test_launch_files.py -v"
    ]
    
    for test in launch_tests:
        success = run_command(test.split(), f"Launch test: {test}")
        if not success:
            return False
    
    return True


def run_ros_tests():
    """Run ROS-specific tests."""
    print("\n🤖 Running ROS Tests")
    
    # Check if ROS environment is available
    try:
        subprocess.run(["ros2", "--help"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ ROS 2 not found. Skipping ROS tests.")
        return True
    
    # Run ROS tests
    ros_tests = [
        "python3 -m pytest test_trajectory_publisher.py -v"
    ]
    
    for test in ros_tests:
        success = run_command(test.split(), f"ROS test: {test}")
        if not success:
            return False
    
    return True


def run_code_quality_tests():
    """Run code quality tests."""
    print("\n📏 Running Code Quality Tests")
    
    # Run flake8
    flake8_success = run_command(
        ["python3", "-m", "flake8", "turtlebot3_tracker/", "--max-line-length=100"],
        "Flake8 code style check"
    )
    
    # Run copyright check
    copyright_success = run_command(
        ["python3", "-m", "ament_copyright", "turtlebot3_tracker/"],
        "Copyright check"
    )
    
    # Run PEP 257 check
    pep257_success = run_command(
        ["python3", "-m", "ament_pep257", "turtlebot3_tracker/"],
        "PEP 257 docstring check"
    )
    
    return flake8_success and copyright_success and pep257_success


def run_all_tests():
    """Run all tests."""
    print("🧪 Running All Tests for turtlebot3_tracker Package")
    print("=" * 60)
    
    tests = [
        ("Unit Tests", run_unit_tests),
        ("Code Quality Tests", run_code_quality_tests),
        ("Integration Tests", run_integration_tests),
        ("Launch Tests", run_launch_tests),
        ("ROS Tests", run_ros_tests),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        print(f"\n{'='*60}")
        print(f"Starting: {test_name}")
        print(f"{'='*60}")
        
        start_time = time.time()
        success = test_func()
        end_time = time.time()
        
        results[test_name] = {
            'success': success,
            'duration': end_time - start_time
        }
        
        status = "✅ PASSED" if success else "❌ FAILED"
        print(f"\n{test_name}: {status} ({end_time - start_time:.2f}s)")
    
    # Print summary
    print("\n" + "="*60)
    print("📊 TEST SUMMARY")
    print("="*60)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results.items():
        status = "✅ PASSED" if result['success'] else "❌ FAILED"
        print(f"{test_name:<25} {status:<10} ({result['duration']:.2f}s)")
        if result['success']:
            passed += 1
    
    print(f"\nOverall: {passed}/{total} test suites passed")
    
    if passed == total:
        print("🎉 All tests passed!")
        return True
    else:
        print("💥 Some tests failed!")
        return False


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Run tests for turtlebot3_tracker package")
    parser.add_argument(
        '--test-type',
        choices=['unit', 'integration', 'launch', 'ros', 'quality', 'all'],
        default='all',
        help='Type of tests to run'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Change to the test directory
    test_dir = Path(__file__).parent
    os.chdir(test_dir)
    
    if args.verbose:
        print(f"Working directory: {os.getcwd()}")
        print(f"Python path: {sys.path}")
    
    # Run selected tests
    if args.test_type == 'unit':
        success = run_unit_tests()
    elif args.test_type == 'integration':
        success = run_integration_tests()
    elif args.test_type == 'launch':
        success = run_launch_tests()
    elif args.test_type == 'ros':
        success = run_ros_tests()
    elif args.test_type == 'quality':
        success = run_code_quality_tests()
    else:  # all
        success = run_all_tests()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main() 