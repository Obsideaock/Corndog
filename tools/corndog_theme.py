#!/usr/bin/env python3
"""
corndog_theme.py — the Corndog dark theme for every Tkinter app on the Pi.

Same palette as the Steam Deck client (near-black background, panel gray,
red accent, green for positive states). Applied through Tk's *option
database*, so existing widgets pick it up automatically at creation time —
no per-widget changes, no functional changes. Sliders stay sliders.

Usage (right after creating the window, before creating widgets):

    from corndog_theme import apply_theme, accent_button, green_button
    window = tk.Tk()
    apply_theme(window)
"""

# ---- palette ---------------------------------------------------------------
# These are the EXACT colors the Steam Deck client displays on screen
# (its palette tuples are OpenCV BGR, so these are the byte-swapped hex).
BG_DARK    = "#17110D"   # window background (warm near-black)
BG_PANEL   = "#221B16"   # buttons / troughs / panels
BG_RAISED  = "#2C231C"   # hover
BORDER     = "#2F2721"   # outlines
ACCENT     = "#C81E1E"   # Corndog red
ACCENT_DIM = "#3C0F0F"   # red-tinted panel (active/pressed)
GREEN      = "#21A179"   # positive / connected
GREEN_DIM  = "#0A2D19"
TEXT_HI    = "#F3EDE6"   # warm white
TEXT_MID   = "#9E948B"   # warm gray
TEXT_ERR   = "#4951F8"   # yes, blue — it's what the Deck really shows

FONT_UI   = "Helvetica"
FONT_MONO = "Courier"


def apply_theme(root):
    """Set the dark theme on a Tk root (or Toplevel-owning app).

    Visual-only: colors, borders, and hover states. All defaults are set at
    lowest priority ('startupFile'-level via option_add), so any explicit
    color a widget sets for itself still wins.
    """
    root.configure(bg=BG_DARK)

    o = root.option_add
    # ---- global defaults ----
    o("*background", BG_DARK)
    o("*foreground", TEXT_HI)
    o("*highlightThickness", 0)
    o("*highlightBackground", BG_DARK)
    o("*insertBackground", TEXT_HI)          # text cursor
    o("*selectBackground", ACCENT_DIM)
    o("*selectForeground", TEXT_HI)
    o("*troughColor", BG_PANEL)

    # ---- buttons ----
    o("*Button.background", BG_PANEL)
    o("*Button.foreground", TEXT_HI)
    o("*Button.activeBackground", BG_RAISED)
    o("*Button.activeForeground", TEXT_HI)
    o("*Button.relief", "flat")
    o("*Button.borderWidth", 1)
    o("*Button.highlightThickness", 1)
    o("*Button.highlightBackground", BORDER)
    o("*Button.highlightColor", ACCENT)

    # ---- labels / frames / panels ----
    o("*Label.background", BG_DARK)
    o("*Label.foreground", TEXT_HI)
    o("*Frame.background", BG_DARK)
    o("*Labelframe.background", BG_DARK)
    o("*Labelframe.foreground", TEXT_MID)     # panel titles
    o("*Labelframe.borderWidth", 1)
    o("*Labelframe.relief", "groove")
    o("*Toplevel.background", BG_DARK)

    # ---- scales (sliders): same widgets, dark clothes ----
    o("*Scale.background", BG_DARK)
    o("*Scale.foreground", TEXT_HI)           # the value readout
    o("*Scale.troughColor", BG_PANEL)
    o("*Scale.activeBackground", BG_RAISED)   # knob on hover
    o("*Scale.highlightThickness", 0)
    o("*Scale.borderWidth", 1)

    # ---- checkbuttons / radiobuttons ----
    o("*Checkbutton.background", BG_DARK)
    o("*Checkbutton.foreground", TEXT_HI)
    o("*Checkbutton.activeBackground", BG_DARK)
    o("*Checkbutton.activeForeground", TEXT_HI)
    o("*Checkbutton.selectColor", BG_PANEL)   # the box itself
    o("*Radiobutton.background", BG_DARK)
    o("*Radiobutton.foreground", TEXT_HI)
    o("*Radiobutton.activeBackground", BG_DARK)
    o("*Radiobutton.selectColor", BG_PANEL)

    # ---- option menus (Menubutton + its dropdown Menu) ----
    o("*Menubutton.background", BG_PANEL)
    o("*Menubutton.foreground", TEXT_HI)
    o("*Menubutton.activeBackground", BG_RAISED)
    o("*Menubutton.activeForeground", TEXT_HI)
    o("*Menubutton.relief", "flat")
    o("*Menubutton.highlightThickness", 1)
    o("*Menubutton.highlightBackground", BORDER)
    o("*Menu.background", BG_PANEL)
    o("*Menu.foreground", TEXT_HI)
    o("*Menu.activeBackground", ACCENT_DIM)
    o("*Menu.activeForeground", TEXT_HI)
    o("*Menu.borderWidth", 1)

    # ---- entries / text (if any get added later) ----
    o("*Entry.background", BG_PANEL)
    o("*Entry.foreground", TEXT_HI)
    o("*Entry.relief", "flat")
    o("*Entry.highlightThickness", 1)
    o("*Entry.highlightBackground", BORDER)
    o("*Entry.highlightColor", ACCENT)
    o("*Text.background", BG_PANEL)
    o("*Text.foreground", TEXT_HI)


# ---- optional helpers: call on individual widgets for emphasis ------------

def accent_button(btn):
    """Make a button the red 'primary action' (e.g. Start Gait)."""
    btn.configure(bg=ACCENT_DIM, fg=TEXT_HI,
                  activebackground=ACCENT, activeforeground=TEXT_HI,
                  highlightbackground=ACCENT, highlightthickness=1)


def green_button(btn):
    """Make a button the green 'safe/positive action' (e.g. Feet -> Home)."""
    btn.configure(bg=GREEN_DIM, fg=TEXT_HI,
                  activebackground=GREEN, activeforeground=BG_DARK,
                  highlightbackground=GREEN, highlightthickness=1)


def status_label(lbl, kind="mid"):
    """Style a status bar label: kind in {'mid','ok','err'}."""
    color = {"mid": TEXT_MID, "ok": GREEN, "err": TEXT_ERR}.get(kind, TEXT_MID)
    lbl.configure(bg=BG_PANEL, fg=color, padx=8, pady=4)
