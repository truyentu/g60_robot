"""
Test script to verify robot package loading via IPC
"""
import zmq
import json
import time
import uuid

def send_request(socket, msg_type, payload=None):
    """Send IPC request and get response"""
    msg = {
        "type": msg_type,
        "id": str(uuid.uuid4()),
        "timestamp": int(time.time() * 1000),
        "payload": payload if payload else {}
    }

    print(f"\n=== Sending {msg_type} ===")
    print(f"Request: {json.dumps(msg, indent=2)}")

    socket.send_json(msg)
    response = socket.recv_json()

    print(f"Response: {json.dumps(response, indent=2)}")
    return response

def main():
    print("Robot Package Loading Test")
    print("=" * 60)

    # Connect to Core IPC
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    print("Connected to Core IPC server")

    # Test 0: GET_STATUS to verify connection
    print("\n" + "=" * 60)
    print("TEST 0: Verify IPC connection with GET_STATUS")
    print("=" * 60)

    resp = send_request(socket, "GET_STATUS")
    if resp.get("type") == "STATUS":
        print("\nOK Core is responding")
    else:
        print(f"\nX Failed to connect to Core: {resp.get('type')}")
        socket.close()
        context.term()
        return

    # Test 1: GET_ROBOT_PACKAGES
    print("\n" + "=" * 60)
    print("TEST 1: Get list of robot packages")
    print("=" * 60)

    resp = send_request(socket, "GET_ROBOT_PACKAGES")
    if resp.get("type") != "ERROR":
        packages = resp.get("payload", {}).get("packages", [])
        print(f"\nOK Found {len(packages)} packages:")
        for pkg in packages:
            print(f"  - {pkg['id']}: {pkg['name']} ({pkg['manufacturer']})")
            print(f"    DOF: {pkg['dof']}, Payload: {pkg['payload_kg']}kg, Reach: {pkg['reach_mm']}mm")
    else:
        print(f"\nX Failed: {resp.get('payload', {}).get('message')}")
        return
        return

    # Test 2: LOAD_ROBOT_PACKAGE kr10r1420
    print("\n" + "=" * 60)
    print("TEST 2: Load KR10 R1420 package")
    print("=" * 60)

    resp = send_request(socket, "LOAD_ROBOT_PACKAGE", {
        "package_id": "kr10r1420"
    })

    if resp.get("type") != "ERROR":
        pkg = resp.get("payload", {})
        print(f"\nOK Package loaded successfully!")
        print(f"  Name: {pkg.get('name')}")
        print(f"  Manufacturer: {pkg.get('manufacturer')}")
        print(f"  Payload: {pkg.get('payload_kg')}kg")
        print(f"  Reach: {pkg.get('reach_mm')}mm")
        print(f"  Joints: {len(pkg.get('joints', []))}")
        print(f"  Package Path: {pkg.get('package_path')}")

        # Check meshes
        print(f"\n  Base mesh: {pkg.get('base_mesh')}")
        for i, joint in enumerate(pkg.get('joints', [])):
            mesh = joint.get('mesh', {}).get('visual_mesh', '')
            print(f"  Joint {i+1} ({joint['name']}): {mesh}")
    else:
        print(f"\nX Failed: {resp.get('payload', {}).get('message')}")
        return

    # Test 3: LOAD_ROBOT_PACKAGE kuka_kr6_r900
    print("\n" + "=" * 60)
    print("TEST 3: Load KR6 R900 package (should also work)")
    print("=" * 60)

    resp = send_request(socket, "LOAD_ROBOT_PACKAGE", {
        "package_id": "kuka_kr6_r900"
    })

    if resp.get("type") != "ERROR":
        pkg = resp.get("payload", {})
        print(f"\nOK Package loaded successfully!")
        print(f"  Name: {pkg.get('name')}")
    else:
        print(f"\nX Failed: {resp.get('payload', {}).get('message')}")

    socket.close()
    context.term()

    print("\n" + "=" * 60)
    print("All tests completed!")
    print("=" * 60)

if __name__ == "__main__":
    main()
