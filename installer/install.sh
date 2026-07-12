#!/usr/bin/env bash
# =============================================================================
#  Corndog installer — sets up everything on a fresh Raspberry Pi.
#
#  Usage (one line, from anywhere):
#     curl -sSL https://raw.githubusercontent.com/Obsideaock/Corndog/main/installer/install.sh | bash
#
#  Or from a cloned repo:
#     bash installer/install.sh
#
#  What it does:
#    1. checks OS / internet, installs apt dependencies
#    2. enables I2C (and the camera stack) via raspi-config
#    3. clones the Corndog repo (default: ~/Corndog) + creates a Python venv
#    4. installs pip deps (piwheels-friendly), spot_micro_kinematics from git,
#       and the LCD driver library (upstream + Corndog overlay)
#    5. installs the `corndog` command into ~/.local/bin
#    6. installs & enables the corndog-supervisor systemd service (StartupLCD)
#    7. points the user at `corndog calibrate` for first-run setup
#
#  Safe to re-run: every step is idempotent (re-running = update/repair).
# =============================================================================
set -euo pipefail

REPO_URL="${CORNDOG_REPO:-https://github.com/Obsideaock/Corndog.git}"
BRANCH="${CORNDOG_BRANCH:-main}"
INSTALL_DIR="${CORNDOG_DIR:-$HOME/Corndog}"
# If this script is being run FROM a cloned repo (bash installer/install.sh),
# install THAT repo in place instead of cloning a second copy to ~/Corndog.
# (When piped via curl, BASH_SOURCE is empty and this is skipped.)
if [ -z "${CORNDOG_DIR:-}" ] && [ -n "${BASH_SOURCE[0]:-}" ] && [ -f "${BASH_SOURCE[0]}" ]; then
    _SELF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    if [ -d "$_SELF_ROOT/.git" ] && [ -f "$_SELF_ROOT/installer/install.sh" ]; then
        INSTALL_DIR="$_SELF_ROOT"
    fi
fi
CONFIG_DIR="$HOME/.config/corndog"
BIN_DIR="$HOME/.local/bin"
# lcd library ships inside the repo (lcd_custom/)
SERVICE_NAME="corndog-supervisor"

RED=$'\e[31m'; GRN=$'\e[32m'; YLW=$'\e[33m'; CYN=$'\e[36m'; RST=$'\e[0m'
step()  { echo; echo "${CYN}==>${RST} ${1}"; }
ok()    { echo "    ${GRN}ok${RST} ${1:-}"; }
warn()  { echo "    ${YLW}!!${RST} ${1}"; }
die()   { echo "${RED}error:${RST} ${1}" >&2; exit 1; }

banner() {
cat <<'EOF'

     ____                     _
    / ___|___  _ __ _ __   __| | ___   __ _
   | |   / _ \| '__| '_ \ / _` |/ _ \ / _` |
   | |__| (_) | |  | | | | (_| | (_) | (_| |
    \____\___/|_|  |_| |_|\__,_|\___/ \__, |
                                      |___/
      quadruped installer — openlonehollow.com

EOF
}

# ---------------------------------------------------------------------------
banner

[ "$(id -u)" -eq 0 ] && die "run as a normal user, not root (sudo is used only where needed)"
command -v sudo >/dev/null || die "sudo is required"

step "Checking system"
if [ -f /proc/device-tree/model ] && grep -qi "raspberry pi" /proc/device-tree/model; then
    ok "$(tr -d '\0' </proc/device-tree/model)"
else
    warn "this doesn't look like a Raspberry Pi — continuing anyway (dev install?)"
fi
ping -c1 -W3 github.com >/dev/null 2>&1 || die "no internet connection"
ok "internet reachable"

step "Installing system packages (sudo)"
sudo apt-get update -qq
sudo apt-get install -y -qq \
    git python3 python3-venv python3-pip python3-tk python3-dev \
    i2c-tools network-manager libopencv-dev libatlas-base-dev \
    libcap-dev python3-libcamera python3-picamera2 >/dev/null || \
    sudo apt-get install -y -qq \
        git python3 python3-venv python3-pip python3-tk python3-dev \
        i2c-tools network-manager libopencv-dev >/dev/null
ok "apt packages installed"

step "Enabling I2C + camera"
if command -v raspi-config >/dev/null; then
    sudo raspi-config nonint do_i2c 0 || warn "could not enable I2C automatically"
    sudo raspi-config nonint do_camera 0 2>/dev/null || true
    ok "I2C enabled"
else
    warn "raspi-config not found — enable I2C manually if needed"
fi
for grp in i2c gpio video dialout; do
    getent group "$grp" >/dev/null && sudo usermod -aG "$grp" "$USER" || true
done
ok "user '$USER' added to hardware groups (takes effect next login)"

step "Fetching Corndog -> $INSTALL_DIR"
if [ -d "$INSTALL_DIR/.git" ]; then
    # Existing repo: respect whatever branch it's on (dev machines live on
    # experimental) — never force-switch it.
    BRANCH="$(git -C "$INSTALL_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "$BRANCH")"
    git -C "$INSTALL_DIR" fetch --quiet origin || warn "fetch failed (offline?)"
    git -C "$INSTALL_DIR" pull --quiet --ff-only origin "$BRANCH" 2>/dev/null || \
        warn "local changes or unpushed work present — skipped pull (this is fine on a dev machine)"
    ok "existing install refreshed (branch: $BRANCH)"
else
    git clone --quiet --branch "$BRANCH" "$REPO_URL" "$INSTALL_DIR"
    ok "repo cloned (branch: $BRANCH)"
fi

step "Python virtual environment"
VENV="$INSTALL_DIR/venv"
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
# --system-site-packages: picamera2/libcamera only exist as apt packages
echo "    (pip may print dependency warnings about unrelated system"
echo "     packages like mkdocs — those are harmless)"
"$VENV/bin/pip" install --quiet --upgrade pip wheel
if [ -f "$INSTALL_DIR/requirements.txt" ]; then
    "$VENV/bin/pip" install --quiet -r "$INSTALL_DIR/requirements.txt"
else
    "$VENV/bin/pip" install --quiet \
        adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit \
        adafruit-circuitpython-bno08x gpiozero numpy opencv-python lgpio
fi
ok "python deps installed"

step "spot_micro_kinematics (IK library)"
# This library is not pip-installable (its setup.py expects a folder layout
# the repo doesn't have) — the intended use is dropping it next to your
# code. So: clone it as third_party/spot_micro_kinematics and point the
# venv at third_party/ with a .pth file.
SMK_DIR="$INSTALL_DIR/third_party/spot_micro_kinematics"
if [ ! -d "$SMK_DIR/.git" ]; then
    mkdir -p "$INSTALL_DIR/third_party"
    git clone --quiet https://github.com/mike4192/spot_micro_kinematics_python.git "$SMK_DIR" \
        || warn "clone failed — IK scripts won't run until fixed"
fi
SITE_DIR="$("$VENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
echo "$INSTALL_DIR/third_party" > "$SITE_DIR/corndog_third_party.pth"
if "$VENV/bin/python" -c "import spot_micro_kinematics" 2>/dev/null; then
    ok "spot_micro_kinematics importable"
else
    warn "spot_micro_kinematics import failed — IK scripts won't run until fixed"
fi

step "LCD driver library"
# lcd_custom/ in the repo is the complete working library (upstream
# the-raspberry-pi-guy/lcd + Corndog's lcd_library helper). The code
# imports it as `lcd`, so mirror it there.
if [ -d "$INSTALL_DIR/lcd_custom" ]; then
    rm -rf "$INSTALL_DIR/lcd"
    cp -r "$INSTALL_DIR/lcd_custom" "$INSTALL_DIR/lcd"
    # lcd_library does `import drivers` resolved from the repo root, and
    # drivers/ is gitignored — materialize it from lcd_custom
    if [ ! -d "$INSTALL_DIR/drivers" ] && [ -d "$INSTALL_DIR/lcd_custom/drivers" ]; then
        cp -r "$INSTALL_DIR/lcd_custom/drivers" "$INSTALL_DIR/drivers"
    fi
    ok "lcd/ + drivers/ in place (from lcd_custom/)"
else
    warn "lcd_custom/ not found in repo — LCD helper missing until next update"
fi
mkdir -p "$INSTALL_DIR/sandbox"   # user playground, never touched by updates

step "Recording install location"
mkdir -p "$CONFIG_DIR"
cat > "$CONFIG_DIR/install.env" <<EOF
CORNDOG_DIR=$INSTALL_DIR
CORNDOG_VENV=$VENV
CORNDOG_BRANCH=$BRANCH
EOF
ok "$CONFIG_DIR/install.env"

step "Installing the 'corndog' command"
mkdir -p "$BIN_DIR"
cp -f "$INSTALL_DIR/installer/corndog" "$BIN_DIR/corndog"
chmod +x "$BIN_DIR/corndog"
case ":$PATH:" in
    *":$BIN_DIR:"*) ;;
    *) warn "$BIN_DIR is not on PATH yet — log out/in once, or run: export PATH=\$PATH:$BIN_DIR" ;;
esac
ok "corndog -> $BIN_DIR/corndog"

step "Supervisor service (StartupLCD at boot)"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=Corndog supervisor (StartupLCD)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$INSTALL_DIR
Environment=CORNDOG_DIR=$INSTALL_DIR
ExecStart=$VENV/bin/python $INSTALL_DIR/StartupLCD.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME" >/dev/null
ok "service installed + enabled at boot (started after reboot)"

step "Passwordless supervisor control"
# `corndog` pauses/resumes the supervisor around every launch. Grant
# passwordless sudo for EXACTLY those three commands (nothing else), so it
# also works from GUI launchers like Geany build commands with no terminal
# password prompt.
SYSTEMCTL_BIN="$(command -v systemctl)"
sudo tee /etc/sudoers.d/corndog-supervisor >/dev/null <<EOF
$USER ALL=(root) NOPASSWD: $SYSTEMCTL_BIN stop $SERVICE_NAME, $SYSTEMCTL_BIN start $SERVICE_NAME, $SYSTEMCTL_BIN restart $SERVICE_NAME
EOF
sudo chmod 440 /etc/sudoers.d/corndog-supervisor
ok "corndog can pause/resume the supervisor without a password prompt"

# ---------------------------------------------------------------------------
echo
echo "${GRN}=============================================${RST}"
echo "${GRN} Corndog is installed.${RST}"
echo
echo "  Next steps:"
echo "    1. reboot once:            ${CYN}sudo reboot${RST}"
echo "       (applies I2C + group changes, starts the supervisor)"
echo "    2. calibrate the servos:   ${CYN}corndog calibrate${RST}"
echo "    3. everything else:        ${CYN}corndog help${RST}"
echo
if [ ! -f "$CONFIG_DIR/calibration.json" ]; then
    echo "  ${YLW}No calibration found yet — the robot won't stand right until"
    echo "  you run 'corndog calibrate'.${RST}"
fi
echo "${GRN}=============================================${RST}"
