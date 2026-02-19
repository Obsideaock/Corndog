#!/usr/bin/env python3
import asyncio
import time
import threading
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

from evdev import InputDevice, list_devices, ecodes, categorize, KeyEvent

import main as bot  # change if needed


# ----------------------------
# Flipper device matching
# ----------------------------
NAME_MATCHES = ("flipper", "keynote", "control")


def is_flipper_dev(path: str) -> Optional[InputDevice]:
    try:
        dev = InputDevice(path)
    except Exception:
        return None
    name = (dev.name or "").lower()
    return dev if any(s in name for s in NAME_MATCHES) else None


# ----------------------------
# LCD writer (non-blocking, coalesced)
# ----------------------------
LCD_WIDTH = 16

def _fit16(s: str) -> str:
    s = (s or "").replace("\n", " ")
    if len(s) >= LCD_WIDTH:
        return s[:LCD_WIDTH]
    return s.ljust(LCD_WIDTH)

class LCDWorker:
    """
    Dedicated thread that performs LCD I/O so evdev handling never blocks.
    Coalesces updates: if you press DOWN 5 times fast, we only write the latest.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = False
        self._pending: Optional[str] = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        self._event.set()
        self._thread.join(timeout=1.0)

    def show_two_lines(self, line1: str, line2: str):
        msg32 = _fit16(line1) + _fit16(line2)  # no newline; 32 chars wraps on 16x2
        with self._lock:
            self._pending = msg32
        self._event.set()

    def clear(self):
        with self._lock:
            self._pending = " " * (LCD_WIDTH * 2)
        self._event.set()

    def _run(self):
        while not self._stop:
            self._event.wait()
            self._event.clear()

            # Grab latest pending message, discard older
            with self._lock:
                msg = self._pending
                self._pending = None

            if msg is None:
                continue

            try:
                bot.lcd.lcd(msg)
            except Exception:
                # Don't crash the controller if LCD glitches
                pass


lcdw = LCDWorker()


# ----------------------------
# Motion step DSL
# ----------------------------
class Step:
    def run(self) -> None:
        raise NotImplementedError


@dataclass
class Move(Step):
    movements: dict
    speed: int = 10
    delay: float = 0.01

    def run(self) -> None:
        bot.move_motors(self.movements, delay=self.delay, speed_multiplier=self.speed)


@dataclass
class Sleep(Step):
    seconds: float

    def run(self) -> None:
        time.sleep(self.seconds)


@dataclass
class Call(Step):
    fn: Callable[[], None]

    def run(self) -> None:
        self.fn()


@dataclass
class Repeat(Step):
    n: int
    steps: Sequence[Step]

    def run(self) -> None:
        for _ in range(self.n):
            for st in self.steps:
                st.run()


def MOVE(movements: dict, *, speed: int = 10, delay: float = 0.01) -> Move:
    return Move(movements=movements, speed=speed, delay=delay)

def SLEEP(seconds: float) -> Sleep:
    return Sleep(seconds=seconds)

def CALL(fn: Callable[[], None]) -> Call:
    return Call(fn)

def REPEAT(n: int, *steps: Step) -> Repeat:
    return Repeat(n=n, steps=list(steps))


@dataclass
class Motion:
    label: str
    steps: Sequence[Step]


# Prevent overlapping actions if center is spammed
MOTION_LOCK = threading.Lock()


def run_motion(m: Motion) -> None:
    with MOTION_LOCK:
        lcdw.show_two_lines(f"RUN: {m.label}", "...")
        try:
            for st in m.steps:
                st.run()
        finally:
            # leave redraw to menu afterwards
            pass


# ----------------------------
# Queue controller
# ----------------------------
class QueueController:
    def __init__(self, motions: List[Motion], loop: asyncio.AbstractEventLoop):
        if not motions:
            raise ValueError("No motions defined.")
        self.motions = motions
        self.loop = loop
        self.index = 0

        # Optional: debounce key spam so a single press doesn't generate multiple moves
        self._last_nav_t = 0.0
        self._nav_min_interval = 0.06  # seconds

    def current(self) -> Motion:
        return self.motions[self.index]

    def next(self) -> Motion:
        return self.motions[(self.index + 1) % len(self.motions)]

    def show(self) -> None:
        cur = self.current().label
        nxt = self.next().label
        lcdw.show_two_lines(f"> {cur}", f"  {nxt}")

    def move_up(self) -> None:
        now = time.time()
        if now - self._last_nav_t < self._nav_min_interval:
            return
        self._last_nav_t = now

        self.index = (self.index - 1) % len(self.motions)
        self.show()

    def move_down(self) -> None:
        now = time.time()
        if now - self._last_nav_t < self._nav_min_interval:
            return
        self._last_nav_t = now

        self.index = (self.index + 1) % len(self.motions)
        self.show()

    async def activate(self) -> None:
        m = self.current()
        await self.loop.run_in_executor(None, lambda: run_motion(m))
        # auto-advance after completion
        self.index = (self.index + 1) % len(self.motions)
        self.show()


# ----------------------------
# Define motions IN ORDER here
# ----------------------------
MOTIONS: List[Motion] = [
	Motion(
		label="Comp-Open",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0.05,0,-0.09), 1:(0.05,0,-0.09), 2:(-0.05,0,-0.09), 3:(-0.05,0,-0.09)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="Comp-Clamp",
		steps=[
			MOVE({4:0, 1:-10, 5:50, 14:-25, 10:10, 15:0, 11:0, 6:-30, 9:35}),
			SLEEP(0.1),
			MOVE({0:-10}),
		],
	),
	Motion(
		label="Comp-Open",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0.05,0,-0.09), 1:(0.05,0,-0.09), 2:(-0.05,0,-0.09), 3:(-0.05,0,-0.09)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="IK-Reset",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0,0,0), 1:(0,0,0), 2:(0,0,0), 3:(0,0,0)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="Sit",
		steps=[
			MOVE({0: -40, 4: 15, 5: -15, 1: 40}),
			MOVE({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10}),
		],
	),
	Motion(
		label="Wave",
		steps=[
			MOVE({10: 50, 14: 80, 8: -20}),
			REPEAT(
				4,
				MOVE({14: 20, 8: 40}),
				SLEEP(0.15),
				MOVE({14: -20, 8: -40}),
				SLEEP(0.15),
			),
			MOVE({10: -50, 14: -40, 8: 20}),
			MOVE({14: -40}),
		],
	),
	Motion(
		label="Unsit",
		steps=[
			MOVE({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10}),
			MOVE({0: 40, 4: -15, 5: 15, 1: -40}),
		],
	),
    Motion(
        label="Dance",
        steps=[
            REPEAT(
                4,
                CALL(lambda: bot.iklegs_move(
                    {0:(0.0,0.0,+0.015),1:(0.0,0.0,-0.015),2:(0.0,0.0,+0.015),3:(0.0,0.0,-0.015)},
                    step_multiplier=10, speed=0.005, delay=0.0
                )),
                SLEEP(0.3),
                CALL(lambda: bot.iklegs_move(
                    {0:(0.0,0.0,-0.015),1:(0.0,0.0,+0.015),2:(0.0,0.0,-0.015),3:(0.0,0.0,+0.015)},
                    step_multiplier=10, speed=0.005, delay=0.0
                )),
                SLEEP(0.3),
            ),
            CALL(lambda: bot.iklegs_move(
                {0:(0.0,0.0,0.0),1:(0.0,0.0,0.0),2:(0.0,0.0,0.0),3:(0.0,0.0,0.0)},
                step_multiplier=10, speed=20, delay=0.0
            )),
        ],
    ),
	Motion(
		label="Sit",
		steps=[
			MOVE({0: -40, 4: 15, 5: -15, 1: 40}),
			MOVE({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10}),
		],
	),
	Motion(
		label="Limp",
		steps=[
			MOVE({10: 95, 11: -95, 4: -50, 5: 50}),
		],
	),
	Motion(
		label="Unlimp",
		steps=[
			MOVE({10: -95, 11: 95, 4: 50, 5: -50}),
		],
	),
	Motion(
		label="Seizure",
		steps=[
			MOVE({0:40, 1:-40, 4:40, 5:40, 6:40, 7:-40, 8:40, 9:-40, 10:40, 11:40, 14:40, 15:-60}),
			SLEEP(2.0),
			MOVE({6:-40, 7:40, 8:-40, 9:40}),
			SLEEP(0.5),
			MOVE({0:-40, 1:40, 4:-40, 5:-40, 10:-40, 11:-40, 14:-40, 15:60}),
		],
	),
	Motion(
		label="Unsit",
		steps=[
			MOVE({15:-90, 14:90, 11:60, 10:-60, 0:-10, 1:10}),
			MOVE({0: 40, 4: -15, 5: 15, 1: -40}),
			CALL(bot.stand_up),
		],
	),
	Motion(
		label="Fetal",
		steps=[
			MOVE({15:-40, 0:-40, 14:40, 1:40, 8:-30, 9:30, 6:-30, 7:30}),
		],
	),
	Motion(
		label="Elder",
		steps=[
			MOVE({8:30, 9:-30, 6:30, 7:-30}),
			MOVE({15:40, 0:40, 14:-40, 1:-40}),
			CALL(bot.stand_up),
		],
	),
	Motion(
		label="Sit",
		steps=[
			MOVE({0: -40, 4: 15, 5: -15, 1: 40}),
			MOVE({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10}),
		],
	),
	Motion(
		label="Sadness",
		steps=[
			MOVE({4: -40, 5: 40, 15: -130, 11:60, 10:-60, 14:130, 8:40, 9:-40}),
		],
	),
	Motion(
		label="Comp-Open",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0.05,0,-0.09), 1:(0.05,0,-0.09), 2:(-0.05,0,-0.09), 3:(-0.05,0,-0.09)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="Comp-Clamp",
		steps=[
			MOVE({4:0, 1:-10, 5:50, 14:-25, 10:10, 15:0, 11:0, 6:-30, 9:35}),
			SLEEP(0.1),
			MOVE({0:-10}),
		],
	),
	Motion(
		label="Comp-Scoop",
		steps=[
			MOVE({9:-35}),
			SLEEP(0.1),
			MOVE({11:-100, 15:80}),
			SLEEP(0.2),
			REPEAT(
				3,
				MOVE({15:-80, 11:20}),
				SLEEP(0.15),
				MOVE({15:80, 11:-20}),
				SLEEP(0.15),
			),
			SLEEP(0.2),
			MOVE({11:100,15:-80}),
			SLEEP(0.1),
			MOVE({9:35}),
		],
	),
	#then sewing?
	Motion(
		label="Comp-Open",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0.05,0,-0.09), 1:(0.05,0,-0.09), 2:(-0.05,0,-0.09), 3:(-0.05,0,-0.09)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="IK-Reset",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0,0,0), 1:(0,0,0), 2:(0,0,0), 3:(0,0,0)},
				step_multiplier=15, speed=10, delay=0.01
			)),
		],
	),
	Motion(
		label="Fetal",
		steps=[
			MOVE({15:-40, 0:-40, 14:40, 1:40, 8:-30, 9:30, 6:-30, 7:30}),
		],
	),
	Motion(
		label="IK-Reset-Sit",
		steps=[
			CALL(lambda: bot.iklegs_move(
				{0:(0,0,0), 1:(0,0,0), 2:(0,0,0), 3:(0,0,0)},
				step_multiplier=15, speed=10, delay=0.01
			)),
			MOVE({0: -40, 4: 15, 5: -15, 1: 40}),
			MOVE({15: 90, 14: -90, 11: -60, 10: 60, 0: 10, 1: -10}),
		],
	),
	Motion(
		label="Hiding",
		steps=[
			MOVE({4: -40, 5: 40, 15: -130, 11:60, 10:-60, 14:130, 8:40, 9:-40}),
			MOVE({11:-130, 10:130}),
		],
	),
	Motion(
		label="Showing",
		steps=[
			MOVE({11:130, 10:-130}),
			MOVE({4: 40, 5: -40, 15: 130, 11:-60, 10:60, 14:-130, 8:-40, 9:40}),
		],
	),
	Motion(
		label="Wave",
		steps=[
			MOVE({10: 50, 14: 80, 8: -20}),
			REPEAT(
				4,
				MOVE({14: 20, 8: 40}),
				SLEEP(0.15),
				MOVE({14: -20, 8: -40}),
				SLEEP(0.15),
			),
			MOVE({10: -50, 14: -40, 8: 20}),
			MOVE({14: -40}),
		],
	),
	
	# Conclusion
]


# ----------------------------
# evdev handling
# ----------------------------
async def handle_device(path: str, menu: QueueController):
    dev = InputDevice(path)
    print(f"Listening on {dev.path} ({dev.name})")

    try:
        dev.grab()
        print("  grabbed device (exclusive)")
    except OSError as e:
        print(f"  could not grab device: {e}")
        print("  (you may need sudo)")

    try:
        async for e in dev.async_read_loop():
            if e.type != ecodes.EV_KEY:
                continue

            ke = categorize(e)

            # Only react on real key presses; ignore holds/releases
            if ke.keystate != KeyEvent.key_down:
                continue

            keycode = ke.keycode
            if isinstance(keycode, list):
                keycode = keycode[0]

            if keycode == "KEY_UP":
                menu.move_up()
            elif keycode == "KEY_DOWN":
                menu.move_down()
            elif keycode in ("KEY_ENTER", "KEY_SPACE"):
                await menu.activate()
    finally:
        try:
            dev.ungrab()
        except OSError:
            pass
        print(f"Released {dev.path}")


async def watch_flipper_devices(menu: QueueController):
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

            # cleanup finished tasks
            for p, t in list(tasks.items()):
                if t.done():
                    tasks.pop(p, None)

            await asyncio.sleep(0.8)
    except KeyboardInterrupt:
        for t in tasks.values():
            t.cancel()


async def main():
    # Start LCD worker
    lcdw.start()
    lcdw.clear()

    # Put robot into known state
    bot.enable_servos()
    bot.stand_up()

    loop = asyncio.get_running_loop()
    menu = QueueController(MOTIONS, loop)
    menu.show()

    print("Queue controller ready.")
    print("UP/DOWN = select, ENTER/SPACE = run (auto-advance).")
    await watch_flipper_devices(menu)


if __name__ == "__main__":
    asyncio.run(main())
