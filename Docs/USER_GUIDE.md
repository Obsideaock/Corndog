# Corndog User Guide — install, update, and every command

## How you use Corndog

Everything happens **on the robot itself**. You remote into your Corndog's
Raspberry Pi — Raspberry Pi Connect is the easiest way (SSH or VNC work
too) — and use its terminal to run commands. If you want to poke at the
code, Geany (preinstalled on Raspberry Pi OS) is right there on the Pi's
desktop.

When you're *not* remoted in, the robot still works on its own: a
supervisor service starts at boot and handles the Steam Deck and Flipper
control modes automatically.

## Installing (one line)

On a fresh Raspberry Pi (Raspberry Pi OS Bookworm 64-bit with desktop),
open a terminal on the Pi and run:

```bash
curl -sSL https://raw.githubusercontent.com/Obsideaock/Corndog/main/installer/install.sh | bash
```

It sets up everything: system packages, I2C, the Python environment, the
`corndog` command, and the boot supervisor. When it finishes:

```bash
sudo reboot
```

That's it. The installer is safe to re-run any time — re-running repairs a
broken install.

## Building & calibrating (the two assembly tools)

**Part A — before assembly, before horns go on:**

```bash
corndog align
```

Holds every servo at the correct angle so you can press the horns on in the
right orientation, and shows which servo plugs into which channel so you
can label the wires.

**Part B — after the robot is fully assembled:**

```bash
corndog calibrate
```

Lay him on his back first (all motors turn on the moment you confirm!).
Nudge each joint until it's perfectly straight / 90 degrees, press `s` to
save, `q` to quit. Done — the robot now knows its true home position
forever. You never need to run this again unless you rebuild a leg.

## Your sandbox (write your own scripts safely)

The folder `~/Corndog/sandbox/` is **yours**. Put your own experiments and
scripts there — updates never touch it, and files there never block an
update. Run them with:

```bash
corndog run sandbox/my_script.py
```

Everything *outside* the sandbox belongs to the official software. You can
read it and learn from it freely, but if you edit those files, `corndog
update` will refuse until you undo the edits or `--force` past them.

## Using Geany (the code editor on the Pi)

If you like pressing Run inside Geany instead of typing commands: open any
Python file, go to **Build → Set Build Commands**, and set the Execute
command to:

```
corndog run "%d/%f"
```

Now F5 runs the current file the proper way — supervisor paused around it,
right Python environment, no password prompts.

## Everyday commands

| Command | What it does |
|---|---|
| `corndog status` | Overview: version, calibration, supervisor state |
| `corndog gui` | On-device control panel |
| `corndog deck` | Steam Deck remote-control mode |
| `corndog gait` | Gait tuning app (sliders for how he walks) |
| `corndog run <file.py>` | Run any script from the repo |
| `corndog supervisor status` | Is the boot service running? |
| `corndog supervisor logs` | Live logs from the boot service |
| `corndog update` | Get the latest Corndog software |
| `corndog uninstall` | Remove the service + command (keeps your files) |

**Always launch scripts through `corndog`** (either a named command or
`corndog run yourscript.py`) rather than plain `python3`. The boot
supervisor owns the servos, and the `corndog` command pauses it
automatically and brings it back when you're done — even if the script
crashes. If you really want to run something by hand:

```bash
corndog supervisor stop
~/Corndog/venv/bin/python yourscript.py
corndog supervisor start
```

## Updating

```bash
corndog update
```

Pulls the newest release and restarts the supervisor. Your calibration and
settings live in `~/.config/corndog/` and are never touched by updates.

If you experimented with the official files in Geany and the update refuses
because of your edits:

```bash
corndog update --force
```

This resets every official file to the release and clears out stray files —
but your `sandbox/` folder and your calibration/settings in
`~/.config/corndog/` always survive. (This is also the "put everything back
the way it should be" button.)

## If something's wrong

1. `corndog status` — is calibration present? Is the supervisor running?
2. `corndog supervisor logs` — watch what the boot service is doing.
3. Re-run the installer — it repairs missing pieces:
   `bash ~/Corndog/installer/install.sh`
4. Servos acting strange? Re-run `corndog calibrate` (on his back!).
5. Broke something while experimenting? `corndog update --force` puts
   every official file back to the release (sandbox + calibration kept).
