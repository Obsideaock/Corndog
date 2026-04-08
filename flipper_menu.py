#!/usr/bin/env python3
import asyncio
from dataclasses import dataclass
from typing import Callable, Dict, List
import time

from evdev import InputDevice, list_devices, ecodes, categorize, KeyEvent

# Import your big robot script. Change this to match the real filename.
# Example: if the big script is saved as "spot_robot_gui.py", use:
#   import spot_robot_gui as bot
import main as bot  # <-- CHANGE ME IF NEEDED


NAME_MATCHES = ("flipper", "keynote", "control")


def is_flipper_dev(path: str):
    """Return an InputDevice for paths that look like the Flipper keyboard."""
    try:
        dev = InputDevice(path)
    except Exception:
        return None
    name = (dev.name or "").lower()
    return dev if any(s in name for s in NAME_MATCHES) else None


@dataclass
class MenuItem:
    label: str
    action: Callable[[], None]
    next_menu: str  # name of the menu to switch to after the action


class MenuController:
    def __init__(self, menus: Dict[str, List[MenuItem]], loop: asyncio.AbstractEventLoop):
        self.menus = menus
        self.loop = loop
        self.current_menu = "main"
        self.index = 0

    def _current_items(self) -> List[MenuItem]:
        return self.menus[self.current_menu]

    def show_current(self):
        items = self._current_items()
        if not items:
            bot.lcd.lcd("(no options)")
            return
        label = items[self.index].label
        bot.lcd.lcd(f"> {label}")

    def move_up(self):
        items = self._current_items()
        if not items:
            return
        self.index = (self.index - 1) % len(items)
        self.show_current()

    def move_down(self):
        items = self._current_items()
        if not items:
            return
        self.index = (self.index + 1) % len(items)
        self.show_current()

    async def activate(self):
        items = self._current_items()
        if not items:
            return
        item = items[self.index]
        action = item.action
        next_menu = item.next_menu

        # Run potentially blocking robot code in a thread pool so we don't
        # block the asyncio event loop (move_motors uses time.sleep, etc).
        if action is not None:
            await self.loop.run_in_executor(None, action)

        # Switch menus (for Sit/Companion contexts, etc.)
        if next_menu in self.menus:
            self.current_menu = next_menu
            self.index = 0

        self.show_current()


# -------------------------------------------------------------------------
# Actions based on your big robot script
# -------------------------------------------------------------------------

def action_stand_up():
    bot.lcd.lcd("Stand up")
    bot.stand_up()
    bot.lcd.clear()


# ----- Sit / Unsit / Shake (same behavior as GUI) ------------------------

def action_sit_enter():
    """Move into the sit pose and switch to the 'sit' submenu."""
    if not bot.is_sitting:
        bot.is_sitting = True
        bot.lcd.lcd("Sitting")
        # Same sequence as your GUI sit() function, just without Tk bits
        bot.move_motors({0: -40, 4: 15, 5: -15, 1: 40})
        bot.move_motors({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10})
        bot.lcd.clear()
    else:
        bot.lcd.lcd("Already sitting")
        time.sleep(0.4)
        bot.lcd.clear()


def action_sit_shake():
    """Shake paw while sitting (copied from arm_shake())."""
    bot.lcd.lcd("Shake!")
    bot.move_motors({10: 130, 14: -80}, speed_multiplier=25)
    for _ in range(4):
        bot.move_motors({14: 40}, speed_multiplier=15)
        time.sleep(0.15)
        bot.move_motors({14: -40}, speed_multiplier=15)
        time.sleep(0.15)
    bot.move_motors({10: -130, 14: 120}, speed_multiplier=25)
    bot.move_motors({14: -40}, speed_multiplier=25)
    bot.lcd.clear()


def action_unsit():
    """Return from sit to normal stand pose (copied from unsit())."""
    if bot.is_sitting:
        bot.is_sitting = False
        bot.lcd.lcd("Standing up")
        bot.move_motors({15: -90, 14: 90, 11: 60, 10: -60, 0: -10, 1: 10})
        bot.move_motors({0: 40, 4: -15, 5: 15, 1: -40})
        bot.stand_up()
        bot.lcd.clear()
    else:
        bot.lcd.lcd("Not sitting")
        time.sleep(0.4)
        bot.lcd.clear()


# ----- Kneel -------------------------------------------------------------

def action_toggle_kneel():
    """Toggle kneel / stand (based on your kneel() function)."""
    if not bot.is_kneeling:
        bot.is_kneeling = True
        bot.lcd.lcd("Kneeling")
        bot.move_motors({15: -30, 11: 30, 10: -30, 14: 30})
        bot.lcd.clear()
    else:
        bot.is_kneeling = False
        bot.lcd.lcd("Standing up")
        bot.move_motors({15: 30, 11: -30, 10: 30, 14: -30})
        bot.stand_up()
        bot.lcd.clear()


# ----- Dance -------------------------------------------------------------

def action_dance():
    """Same as your dance() function."""
    bot.lcd.lcd("Dancing")
    for _ in range(4):
        bot.move_motors({15: -30, 0: -30, 14: 30, 1: 30})
        bot.move_motors({15: 30, 0: 30, 14: -30, 1: -30})
    bot.lcd.clear()


# ----- Jump --------------------------------------------------------------

def action_jump():
    """Same as your jump() function."""
    bot.lcd.lcd("Charging jump")
    bot.iklegs_move(
        {0: (0, 0, 0.03), 1: (0, 0, 0.03), 2: (0, 0, 0.03), 3: (0, 0, 0.03)},
        step_multiplier=20,
        speed=0.05,
    )
    time.sleep(1)
    bot.lcd.lcd("Jumping")
    bot.iklegs_move(
        {0: (0, 0, -0.03), 1: (0, 0, -0.03), 2: (0, 0, -0.03), 3: (0, 0, -0.03)},
        step_multiplier=10,
        speed=50,
        delay=0,
    )
    time.sleep(0.2)
    bot.iklegs_move(
        {0: (0, 0, 0), 1: (0, 0, 0), 2: (0, 0, 0), 3: (0, 0, 0)},
        step_multiplier=10,
        speed=50,
        delay=0,
    )
    bot.lcd.clear()


# ----- Lie Down / Power Off ---------------------------------------------

def action_lie_down():
    """Lie down and disable servos (power_off())."""
    bot.lcd.lcd("Lying down")
    bot.move_motors({15: -40, 0: -40, 14: 40, 1: 40})
    bot.disable_servos()
    bot.lcd.clear()


# -------------------------------------------------------------------------
# Companion flow (mirrors your Companion stages + shake)
# -------------------------------------------------------------------------

def action_companion_start():
    """
    Stage 1: 'Companion: Open'
    Called from main menu 'Companion'. After this, we switch to menu 'companion2'
    which lets you select 'Companion 2'.
    """
    bot.lcd.lcd("Companion: Open")
    bot.iklegs_move(
        {
            0: (0.05, 0, -0.09),
            1: (0.05, 0, -0.09),
            2: (-0.05, 0, -0.09),
            3: (-0.05, 0, -0.09),
        },
        step_multiplier=15,
        speed=10,
        delay=0.01,
    )
    bot.lcd.clear()


def action_companion_stage2():
    """
    Stage 2: 'Companion: Set'.
    In your GUI this then shows Companion 3 + Shake.
    Here, after this runs, menu switches to 'companion3'.
    """
    bot.lcd.lcd("Companion: Set")
    bot.move_motors({4: 0, 1: -10, 5: 50, 14: -25, 10: 10, 15: 0, 11: 0, 6: -30, 9: 35})
    time.sleep(0.1)
    bot.move_motors({0: -10})
    bot.lcd.clear()


def action_companion_shake():
    """
    Companion Shake (compan_shake in your GUI).
    Menu stays on 'companion3' after this so you can shake repeatedly.
    """
    bot.lcd.lcd("Companion: Shake")
    bot.move_motors({9: -35})
    time.sleep(0.1)
    bot.move_motors({11: -100})
    time.sleep(0.2)
    for _ in range(3):
        bot.move_motors({15: -40}, speed_multiplier=15)
        time.sleep(0.15)
        bot.move_motors({15: 40}, speed_multiplier=15)
        time.sleep(0.15)
    time.sleep(0.2)
    bot.move_motors({11: 100})
    time.sleep(0.1)
    bot.move_motors({9: 35})
    bot.lcd.clear()


def action_companion_stage3():
    """
    Stage 3: 'Companion: Closed'.
    After this we show only 'Reset Companion'.
    """
    bot.lcd.lcd("Companion: Closed")
    bot.iklegs_move(
        {
            0: (0.05, 0, -0.09),
            1: (0.05, 0, -0.09),
            2: (-0.05, 0, -0.09),
            3: (-0.05, 0, -0.09),
        },
        step_multiplier=15,
        speed=10,
        delay=0.01,
    )
    bot.lcd.clear()


def action_companion_reset():
    """
    Reset from Companion flow back to neutral (companion_reset).
    After this, menu returns to 'main'.
    """
    bot.lcd.lcd("Companion: Reset")
    bot.iklegs_move(
        {
            0: (0, 0, 0),
            1: (0, 0, 0),
            2: (0, 0, 0),
            3: (0, 0, 0),
        },
        step_multiplier=15,
        speed=10,
        delay=0.01,
    )
    bot.lcd.clear()


# -------------------------------------------------------------------------
# Menu layout
# -------------------------------------------------------------------------

MENUS: Dict[str, List[MenuItem]] = {
    # Main menu: what you see first
    "main": [
        MenuItem("Stand Up",          action_stand_up,      "main"),
        MenuItem("Sit",               action_sit_enter,     "sit"),
        MenuItem("Kneel / Unkneel",   action_toggle_kneel,  "main"),
        MenuItem("Dance",             action_dance,         "main"),
        MenuItem("Jump",              action_jump,          "main"),
        MenuItem("Companion",         action_companion_start, "companion2"),
        MenuItem("Lie Down",          action_lie_down,      "main"),
    ],

    # Sit context menu: only Unsit + Shake available
    "sit": [
        MenuItem("Shake",   action_sit_shake, "sit"),
        MenuItem("Unsit",   action_unsit,     "main"),
    ],

    # Companion stage menus:
    # After selecting 'Companion' from main, stage 1 runs and we go here.
    # Only option: Companion 2 (stage 2).
    "companion2": [
        MenuItem("Companion 2", action_companion_stage2, "companion3"),
    ],

    # After stage 2, we show both Shake & Companion 3 (stage 3).
    "companion3": [
        MenuItem("Shake",        action_companion_shake,  "companion3"),
        MenuItem("Companion 3",  action_companion_stage3, "companion_reset"),
    ],

    # After stage 3, only reset, then back to main.
    "companion_reset": [
        MenuItem("Reset Companion", action_companion_reset, "main"),
    ],
}


# -------------------------------------------------------------------------
# Flipper / evdev handling
# -------------------------------------------------------------------------

async def handle_device(path: str, menu: MenuController):
    dev = InputDevice(path)
    print(f"Listening on {dev.path} ({dev.name})")
    try:
        dev.grab()
        print("  grabbed device (exclusive: keys won't hit the OS)")
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

            keycode = ke.keycode
            if isinstance(keycode, list):
                keycode = keycode[0]

            if keycode == "KEY_UP":
                menu.move_up()
            elif keycode == "KEY_DOWN":
                menu.move_down()
            elif keycode in ("KEY_SPACE", "KEY_ENTER"):
                await menu.activate()
    finally:
        try:
            dev.ungrab()
            print(f"Released {dev.path}")
        except OSError:
            pass


async def watch_flipper_devices(menu: MenuController):
    """Continuously watch for Flipper-style devices appearing / disappearing."""
    tasks = {}
    try:
        while True:
            for p in list_devices():
                if p in tasks:
                    continue
                dev = is_flipper_dev(p)
                if dev:
                    print(f"Found flipper/keynote device: {dev.path} ({dev.name})")
                    tasks[p] = asyncio.create_task(handle_device(dev.path, menu))

            # Drop finished tasks
            for p, t in list(tasks.items()):
                if t.done():
                    tasks.pop(p, None)

            await asyncio.sleep(2)
    except KeyboardInterrupt:
        print("Stopping...")
        for t in tasks.values():
            t.cancel()


async def main():
    # Bring the robot into a known state
    bot.enable_servos()
    bot.stand_up()

    loop = asyncio.get_running_loop()
    menu = MenuController(MENUS, loop)

    # Show the first menu entry on the LCD
    menu.show_current()
    print("Flipper robot menu ready. Use UP/DOWN to pick an action, SPACE to run it.")

    await watch_flipper_devices(menu)


if __name__ == "__main__":
    asyncio.run(main())
