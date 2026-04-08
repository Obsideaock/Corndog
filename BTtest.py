#!/usr/bin/env python3
import asyncio
from evdev import InputDevice, list_devices, ecodes, categorize, KeyEvent

# Match anything that looks like the Flipper keyboard / keynote app
NAME_MATCHES = ("flipper", "keynote", "control")

def is_flipper_dev(path):
    try:
        dev = InputDevice(path)
    except Exception:
        return None
    name = (dev.name or "").lower()
    return dev if any(s in name for s in NAME_MATCHES) else None

# Friendly names for common keys you'll likely use
FRIENDLY_KEY_NAMES = {
    "KEY_UP": "up arrow",
    "KEY_DOWN": "down arrow",
    "KEY_LEFT": "left arrow",
    "KEY_RIGHT": "right arrow",
    "KEY_SPACE": "space",
    "KEY_ENTER": "enter",
    "KEY_ESC": "escape",
    "KEY_PAGEDOWN": "page down",
    "KEY_PAGEUP": "page up",
    "KEY_F5": "F5",
    "KEY_F7": "F7",
}

def pretty_key_name(code):
    key_name = ecodes.KEY.get(code, f"KEY_{code}")
    return FRIENDLY_KEY_NAMES.get(key_name, key_name.lower())

async def handle_device(path):
    dev = InputDevice(path)
    print(f"listening on {dev.path} ({dev.name})")

    # Try to take exclusive control of the device
    try:
        dev.grab()
        print("  grabbed device (exclusive access; keys won't hit the OS)")
    except OSError as e:
        print(f"  could not grab device: {e}")
        print("  (you may need to run this script with sudo)")

    try:
        async for e in dev.async_read_loop():
            if e.type != ecodes.EV_KEY:
                continue

            ke = categorize(e)

            # Only react on key down (press)
            if ke.keystate != KeyEvent.key_down:
                continue

            human_name = pretty_key_name(e.code)
            print(human_name)

    finally:
        # Make sure we release the device on exit
        try:
            dev.ungrab()
            print(f"released {dev.path}")
        except OSError:
            pass

async def main():
    tasks = {}
    try:
        while True:
            # Add any new matching devices
            for p in list_devices():
                if p in tasks:
                    continue
                dev = is_flipper_dev(p)
                if dev:
                    print(f"Found flipper/keynote device: {dev.path} ({dev.name})")
                    tasks[p] = asyncio.create_task(handle_device(p))

            # Drop finished tasks
            for p, t in list(tasks.items()):
                if t.done():
                    tasks.pop(p, None)

            await asyncio.sleep(2)
    except KeyboardInterrupt:
        for t in tasks.values():
            t.cancel()

if __name__ == "__main__":
    asyncio.run(main())
