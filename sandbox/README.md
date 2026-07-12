# Sandbox — your folder

Anything you put in here is YOURS. Updates never touch it, `corndog update
--force` never deletes it, and it won't show up as "local changes" that
block updates.

Start by copying the example (it shows every movement command):

    cp sandbox/example.py sandbox/mytest.py

Then write your experiments and run them with:

    corndog run sandbox/mytest.py

Everything in the main Corndog folders belongs to the official software —
if you edit those files, updates will refuse until you either undo your
edits or run `corndog update --force` (which resets them). This folder is
the safe place instead. (One exception: `example.py` here belongs to
the official software — copy it rather than editing it directly.)
