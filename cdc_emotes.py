"""
cdc_emotes.py  —  Shared emote registry (COPY THIS FILE TO BOTH MACHINES)

This is the single source of truth for which emotes exist, what they're
labelled on the wheel, and what token travels over the wire. Both sides import
it so the wheel and the Pi-side dispatch can never drift out of sync.

  Steam Deck:  draws the wheel from EMOTES, sends "EMOTE <token>".
  Raspberry Pi: maps <token> -> a MoveLib call (that mapping lives on the Pi,
                because only the Pi has the hardware; see corndog_pi_emotes.py).

Keep it dependency-free (no cv2, no hardware imports) so it's safe on both.

Ordering == wheel order, starting at the 12 o'clock position and going
clockwise. Reorder freely; the wheel and dispatch both follow this list.
"""

# (token, label)
#   token : short, lowercase, no spaces — what goes over the socket
#   label : what the user sees on the wheel
EMOTES = [
    ("stand",     "Stand"),      # universal "return to neutral" (exits any pose)
    ("sit",       "Sit"),
    ("kneel",     "Kneel"),
    ("wave",      "Wave"),
    ("shake",     "Shake"),
    ("dance",     "Dance"),
    ("handstand", "Handstand"),
    ("jump",      "Jump"),
    ("fetal",     "Fetal"),
    ("seizure",   "Seizure"),
    ("limp",      "Limp"),
    ("companion", "Companion"),
    ("liedown",   "Lie Down"),
]

# Convenience lookups
EMOTE_TOKENS = [t for t, _ in EMOTES]
EMOTE_LABELS = {t: lbl for t, lbl in EMOTES}


def label_for(token: str) -> str:
    return EMOTE_LABELS.get(token, token)


def is_valid(token: str) -> bool:
    return token in EMOTE_LABELS
