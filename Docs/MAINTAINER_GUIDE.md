# Corndog Maintainer Guide (for Leo)

How this setup thinks about YOUR workflow:

- **Your dog is the dev machine.** You remote in (Pi Connect), edit with
  Geany or the terminal, and run things right there. Your working copy in
  `~/Corndog` is the source of truth — you never *pull* updates, you only
  *push* them.
- Your dog lives on the `experimental` branch. Users live on `main`.
- `corndog update` is a **users-only** command. You are upstream; there is
  nothing for you to update from.
- Your live edits are what runs: `corndog gui`, `corndog run`, etc. launch
  the files in your working copy (with the supervisor auto-paused), so the
  edit → test loop is just "save in Geany, run the command."

---

## 1. DO THIS RIGHT NOW (one-time setup, on the dog)

```bash
cd ~/Corndog

# 1. sanity check — should show installer/, tools/, lcd_custom/, Docs/,
#    requirements.txt, and edits to main.py, MoveLib.py, StartupLCD.py,
#    .gitignore
git status

# 2. commit and push everything to main
git add -A
git commit -m "Add installer, calibration tools, config system, guides"
git push origin main

# 3. create your experimental branch and stay on it (one time only)
git checkout -b experimental
git push -u origin experimental

# 4. run the installer over your existing folder (safe — it detects the
#    repo and adds the venv, service, and corndog command around it)
bash ~/Corndog/installer/install.sh

# 5. tell the launcher your dog tracks experimental (so `corndog status`
#    reports the right thing — you still never run `corndog update`)
sed -i 's/^CORNDOG_BRANCH=.*/CORNDOG_BRANCH=experimental/' ~/.config/corndog/install.env

# 6. reboot once (hardware group changes + supervisor service start)
sudo reboot

# 7. after remoting back in, check everything
corndog status
```

Notes:

- **Remove your old StartupLCD autostart.** The installer added a systemd
  service for it; whatever you used before (crontab, rc.local, an autostart
  .desktop file) must go, or two copies will fight over the GPIO.
  (`crontab -e`, check `/etc/rc.local`, check `~/.config/autostart/`.)
- Your old venv (`robo`) is untouched. The install made its own at
  `~/Corndog/venv`. Delete `robo` whenever you're confident.
- **If `git push` asks for a password:** GitHub needs a token or SSH key on
  the dog (one time). Easiest is the GitHub CLI:
  ```bash
  sudo apt install gh
  gh auth login        # pick GitHub.com -> HTTPS -> login with browser
  ```
  After that, pushes from the dog just work.

---

## 2. Day-to-day development (all on the dog)

You're always on `experimental` (`corndog status` confirms). The loop is:

```bash
# edit in Geany / terminal ... then test immediately:
corndog gui                      # or corndog run tests/whatever.py
# the command runs YOUR edited files, supervisor paused around it

# happy with it? snapshot it:
git add -A
git commit -m "what you changed"
git push origin experimental
```

That's the whole loop. Commit small and often — every commit is a restore
point (`git log --oneline` to see them, `git checkout <hash> -- file.py`
to bring an old version of a file back).

Two special cases:

- **Geany build command:** replace your old Execute command
  (`.../robo/bin/python3.11 "%f"`) with:
  ```
  corndog run "%d/%f"
  ```
  (Build → Set Build Commands → Execute.) F5 now runs whatever file you're
  editing through the launcher: correct venv, supervisor auto-paused, no
  password prompt (the installer grants passwordless sudo for exactly the
  three supervisor commands and nothing else). Works for files anywhere in
  the repo, including `sandbox/`.

- **You edited `installer/corndog` itself** (the launcher): the copy in
  `~/.local/bin` doesn't update by itself for you. Refresh it:
  ```bash
  cp ~/Corndog/installer/corndog ~/.local/bin/corndog
  ```
- **You edited `installer/install.sh`:** nothing to do — users always fetch
  it fresh from GitHub via the curl line.

---

## 3. Releasing to users: `experimental` -> `main`

When a feature is solid, still on the dog:

```bash
cd ~/Corndog
git checkout main
git merge experimental           # bring in your tested work
git push origin main
git checkout experimental        # IMPORTANT: come back to your branch
git merge main                   # keep experimental current (fast-forward)
git push origin experimental
```

Heads-up: for the minute you're on `main`, the code on the dog IS main —
don't do the branch dance mid-drive-session. Coming back to `experimental`
restores your world exactly.

Users get the release next time they run `corndog update`. Their
calibration in `~/.config/corndog/` is never touched.

Hotfix that must skip experimental (rare):

```bash
git checkout main
# ... fix ...
git add -A && git commit -m "hotfix: ..." && git push origin main
git checkout experimental && git merge main && git push origin experimental
```

---

## 4. Generating the master tables (one-time, on your calibrated dog)

```bash
corndog run tools/master_finder.py
```

He snaps to HOME on confirm (on his back!). Nudge every joint until he's in
the reference pose (all straight / 90 deg), press `g`, quit. It prints a
complete replacement for `tools/master_tables.py` and saves a copy at
`~/.config/corndog/master_tables_generated.py`. Paste it over
`tools/master_tables.py` in Geany, sanity-check the theta estimates (clean
numbers like 0 / ±90 if the pose is true — if they're not clean, the current
IK mapping is suspect and worth a look), then commit and push. From that
moment `corndog align` and `corndog calibrate` are fully armed for builders.

---

## 5. Cheat sheet

| I want to... | Command (on the dog) |
|---|---|
| test my current edits | `corndog gui` / `corndog run <file>` — edits are live |
| snapshot work-in-progress | `git add -A && git commit -m "..." && git push origin experimental` |
| release to everyone | section 3 branch dance |
| see branch / commit / service / calibration | `corndog status` |
| pause / resume the supervisor manually | `corndog supervisor stop` / `start` |
| watch supervisor logs | `corndog supervisor logs` |
| refresh the launcher after editing it | `cp installer/corndog ~/.local/bin/corndog` |
