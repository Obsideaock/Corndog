#!/usr/bin/env python3
"""
BLE Serial bridge for Corndog controller.

Protocol (Flipper -> Pi):
  'U' = navigate up
  'D' = navigate down
  'E' = enter/run motion
  'I' + <byte> = sync index (sent on BLE connect)

Both sides auto-advance after ENTER so they stay in lockstep.
On reconnect, Flipper sends its index and Pi adjusts to match.

Menu is initialised only after the first successful BLE connection so that
warm-reconnect / stand-up happens at the right time, not at script launch.
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner

DEVICE_ADDRESS = None
DEVICE_NAME_MATCHES = ("flipper", "control", "corndog", "laon")
FLIPPER_TX_CHAR = "19ed82ae-ed21-4c9d-4145-228e61fe0000"
FLIPPER_RX_CHAR = "19ed82ae-ed21-4c9d-4145-228e62fe0000"

menu       = None
async_loop = None
ble_client = None
lcdw       = None   # set after FlipperTed import
_QueueController = None
_MOTIONS         = None


async def send_sync(index: int):
    global ble_client
    if not ble_client or not ble_client.is_connected:
        return
    data = bytes([ord('S'), index & 0xFF])
    try:
        await ble_client.write_gatt_char(FLIPPER_RX_CHAR, data, response=False)
        print(f"  -> Sync sent: index={index}")
    except Exception as e:
        print(f"  -> Sync write failed: {e}")


rx_state = 0        # parser state: 0=idle, 1=expecting index byte after 'I'
_menu_initializing = False  # guard against double-init if two 'I' bytes arrive fast


def handle_byte(b):
    global rx_state, menu, _menu_initializing

    if rx_state == 1:
        rx_state = 0
        index = b

        if menu is None:
            # First real proof the Flipper app is open — initialise now
            if _menu_initializing or _QueueController is None or async_loop is None:
                return
            _menu_initializing = True
            received_index = index  # capture for closure

            async def init_and_sync():
                global menu, _menu_initializing
                def do_init():
                    m = _QueueController(_MOTIONS, async_loop)
                    m.index = received_index % len(m.motions)
                    m.show()
                    return m
                menu = await async_loop.run_in_executor(None, do_init)
                _menu_initializing = False
                print(f"  Menu initialised at: {menu.current().label}")

            asyncio.run_coroutine_threadsafe(init_and_sync(), async_loop)
            return

        # Menu already exists — just sync index
        num = len(menu.motions)
        index = index % num
        if menu.index != index:
            print(f"BLE: SYNC {menu.motions[menu.index].label} -> {menu.motions[index].label}")
            menu.index = index
            menu.show()
        return

    cmd = chr(b)
    if cmd == 'U':
        print("BLE: UP")
        if menu: menu.move_up()
    elif cmd == 'D':
        print("BLE: DOWN")
        if menu: menu.move_down()
    elif cmd == 'E':
        print("BLE: ENTER")
        if menu and async_loop:
            async def run_motion():
                try:
                    print(f"  Running: {menu.current().label}")
                    await menu.activate()
                    print(f"  Done. Next: {menu.current().label}")
                except Exception as e:
                    print(f"  Motion error: {e}")
                    menu.index = (menu.index + 1) % len(menu.motions)
                    menu.show()
                await send_sync(menu.index)
            asyncio.run_coroutine_threadsafe(run_motion(), async_loop)
    elif cmd == 'I':
        rx_state = 1
    else:
        pass


def on_notify(sender, data: bytearray):
    for b in data:
        handle_byte(b)


async def find_flipper() -> str:
    print("Scanning for Flipper BLE device...")
    while True:
        devices = await BleakScanner.discover(timeout=5.0)
        for dev in devices:
            name = (dev.name or "").lower()
            if any(s in name for s in DEVICE_NAME_MATCHES):
                print(f"  Found: {dev.name} ({dev.address})")
                return dev.address
        await asyncio.sleep(2)


async def connect_and_listen(address: str):
    global ble_client, menu

    print(f"Connecting to {address}...")

    try:
        client = BleakClient(address, timeout=20.0)
        await client.connect()
    except Exception as e:
        print(f"  Connect failed: {e}")
        return

    if not client.is_connected:
        return

    ble_client = client
    print(f"  Connected!")

    try:
        await client.start_notify(FLIPPER_TX_CHAR, on_notify)
        print(f"  Subscribed to Flipper TX")
    except Exception as e:
        print(f"  Subscribe failed: {e}")
        await client.disconnect()
        return

    print("Listening for Flipper commands...")
    print(f"  Waiting for Flipper to send its index to initialise...")

    try:
        while client.is_connected:
            await asyncio.sleep(0.5)
    except Exception:
        pass
    finally:
        ble_client = None
        try:
            await client.disconnect()
        except Exception:
            pass

    print("Disconnected.")

    # Clear the LCD so it doesn't show stale menu text while searching
    if lcdw:
        try:
            lcdw.clear()
        except Exception:
            pass


async def ble_loop():
    while True:
        try:
            address = DEVICE_ADDRESS or await find_flipper()
            await connect_and_listen(address)
        except Exception as e:
            print(f"BLE error: {e}")
        print("Reconnecting in 3s...")
        await asyncio.sleep(3)


async def main_standalone():
    print("=== Corndog BLE Bridge (standalone) ===")
    await ble_loop()


async def main_with_robot():
    global async_loop, lcdw, _QueueController, _MOTIONS

    from Flipperrmrrf import QueueController, MOTIONS, lcdw as _lcdw
    lcdw = _lcdw
    _QueueController = QueueController
    _MOTIONS = MOTIONS

    async_loop = asyncio.get_running_loop()
    lcdw.start()
    lcdw.clear()

    # Menu is intentionally NOT created here — it's created inside
    # connect_and_listen() on the first successful BLE connection so that
    # warm-reconnect / stand-up runs only after the Flipper is actually there.
    print("Robot ready via BLE.")
    await ble_loop()


if __name__ == "__main__":
    if "--address" in sys.argv:
        idx = sys.argv.index("--address")
        if idx + 1 < len(sys.argv):
            DEVICE_ADDRESS = sys.argv[idx + 1]

    if "--name" in sys.argv:
        idx = sys.argv.index("--name")
        if idx + 1 < len(sys.argv):
            DEVICE_NAME_MATCHES = DEVICE_NAME_MATCHES + (sys.argv[idx + 1].lower(),)

    standalone = "--standalone" in sys.argv
    print("=== Corndog BLE Bridge ===")

    if standalone:
        asyncio.run(main_standalone())
    else:
        try:
            asyncio.run(main_with_robot())
        except ImportError as e:
            print(f"Robot not found ({e}), standalone mode.")
            asyncio.run(main_standalone())
