"""
Test script to trigger STL loading via IPC and monitor behavior
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
    socket.send_json(msg)
    response = socket.recv_json()
    return response

def main():
    print("STL Loading Test via IPC")
    print("=" * 60)

    # Connect to Core
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    print("\n1. Testing GET_ROBOT_PACKAGES...")
    resp = send_request(socket, "GET_ROBOT_PACKAGES")

    if resp.get("type") != "ERROR":
        packages = resp.get("payload", {}).get("packages", [])
        print(f"   Found {len(packages)} packages")
        for pkg in packages:
            print(f"   - {pkg['id']}: {pkg['name']}")
    else:
        print(f"   ERROR: {resp.get('payload', {}).get('message')}")
        return

    print("\n2. Testing LOAD_ROBOT_PACKAGE kr10r1420...")
    resp = send_request(socket, "LOAD_ROBOT_PACKAGE", {
        "package_id": "kr10r1420"
    })

    if resp.get("type") != "ERROR":
        pkg = resp.get("payload", {}).get("package", {})
        print(f"   Package: {pkg.get('name')}")
        print(f"   Package Path: {pkg.get('package_path')}")
        print(f"   Base Mesh: {pkg.get('base_mesh')}")

        print(f"\n   Joints ({len(pkg.get('joints', []))}):")
        for i, joint in enumerate(pkg.get('joints', [])):
            mesh = joint.get('mesh', {}).get('visual_mesh', '')
            print(f"   - Joint {i+1} ({joint['name']}): {mesh}")

        # Check if mesh paths would resolve
        import os
        pkg_path = pkg.get('package_path', '')

        print(f"\n3. Checking if STL files exist from Core's perspective...")
        print(f"   Package path: {pkg_path}")

        # Check from build/bin/Debug perspective
        base_dir = "E:\\DEV_CONTEXT_PROJECTs\\Robot_controller\\build\\bin\\Debug"
        full_pkg_path = os.path.join(base_dir, pkg_path)

        print(f"   Full package path: {full_pkg_path}")
        print(f"   Path exists: {os.path.exists(full_pkg_path)}")

        if os.path.exists(full_pkg_path):
            base_mesh_path = os.path.join(full_pkg_path, pkg.get('base_mesh', ''))
            print(f"\n   Base mesh: {base_mesh_path}")
            print(f"   Exists: {os.path.exists(base_mesh_path)}")

            for i, joint in enumerate(pkg.get('joints', [])):
                mesh = joint.get('mesh', {}).get('visual_mesh', '')
                if mesh:
                    mesh_path = os.path.join(full_pkg_path, mesh)
                    exists = os.path.exists(mesh_path)
                    size = os.path.getsize(mesh_path) if exists else 0
                    print(f"   Joint {i+1}: {mesh_path}")
                    print(f"            Exists: {exists}, Size: {size} bytes")

    else:
        print(f"   ERROR: {resp.get('payload', {}).get('message')}")

    socket.close()
    context.term()

    print("\n" + "=" * 60)
    print("Test completed!")

if __name__ == "__main__":
    main()
