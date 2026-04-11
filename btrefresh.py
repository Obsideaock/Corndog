#!/usr/bin/env python3
"""
Quick pair script for Flipper Zero BLE.
Automates: remove -> scan -> pair (with PIN) -> trust -> disconnect
"""

import subprocess
import time
import sys

DEVICE_NAME_MATCHES = ("flipper", "control", "corndog", "laon")
KNOWN_ADDRESS = "80:E1:26:F1:E2:00"  # fallback if scan doesn't find it


def run_bluetoothctl(*commands, timeout=10):
    """Send commands to bluetoothctl and return output."""
    proc = subprocess.Popen(
        ["bluetoothctl"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    input_str = "\n".join(commands) + "\n"
    try:
        out, _ = proc.communicate(input=input_str, timeout=timeout)
        return out
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
        return out


def main():
    address = KNOWN_ADDRESS

    # Allow override via command line
    if len(sys.argv) > 1:
        address = sys.argv[1]

    print(f"=== Flipper BLE Pair Tool ===")
    print()

    # Step 1: Remove old pairing
    print(f"[1/5] Removing old pairing for {address}...")
    out = run_bluetoothctl(f"remove {address}", "quit", timeout=5)
    if "not available" in out:
        print("  No old pairing found (that's fine)")
    else:
        print("  Removed.")
    time.sleep(1)

    # Step 2: Scan
    print(f"[2/5] Scanning for 5 seconds...")
    out = run_bluetoothctl("power on", "scan on", timeout=8)

    # Wait and scan
    time.sleep(5)
    out = run_bluetoothctl("scan off", "devices", "quit", timeout=5)

    # Check if we can find the device
    found = False
    for line in out.split("\n"):
        if address.upper() in line.upper():
            found = True
            print(f"  Found: {line.strip()}")
            break
        for name in DEVICE_NAME_MATCHES:
            if name in line.lower():
                # Extract address from "Device XX:XX:XX:XX:XX:XX Name"
                parts = line.strip().split()
                for p in parts:
                    if ":" in p and len(p) == 17:
                        address = p
                        found = True
                        print(f"  Found: {line.strip()}")
                        break
            if found:
                break

    if not found:
        print(f"  Device not found in scan, trying {address} anyway...")
    print()

    # Step 3: Pair (interactive - needs PIN input)
    print(f"[3/5] Pairing with {address}...")
    print(f"  >>> Look at your Flipper screen for the PIN <<<")
    print()

    proc = subprocess.Popen(
        ["bluetoothctl"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    proc.stdin.write(f"pair {address}\n")
    proc.stdin.flush()

    # Wait for PIN prompt
    time.sleep(3)

    pin = input("  Enter the PIN shown on Flipper: ").strip()

    proc.stdin.write(f"{pin}\n")
    proc.stdin.flush()
    time.sleep(3)

    # Step 4: Trust
    print(f"\n[4/5] Trusting {address}...")
    proc.stdin.write(f"trust {address}\n")
    proc.stdin.flush()
    time.sleep(2)

    # Step 5: Disconnect and quit
    print(f"[5/5] Disconnecting...")
    proc.stdin.write(f"disconnect {address}\n")
    proc.stdin.flush()
    time.sleep(1)
    proc.stdin.write("quit\n")
    proc.stdin.flush()
    try:
        proc.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()

    print()
    print(f"Done! {address} is paired and trusted.")
    print(f"You can now run: python ble_bridge.py")


if __name__ == "__main__":
    main()
