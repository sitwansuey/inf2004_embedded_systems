# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID or demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row. The core system tests each need
one filmed run and a link in the test result. Put the commit in backticks
and the video link in angle brackets.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-11 | System | SYS-07 | `af11df6` or `1d088b2` | 2500238 | <https://youtu.be/b28la7W1juI> | Barcode turns: Z mapped to Left, then Right; A mapped to U-turn |
| 2026-11-20 | System | SYS-03 | `89f1bdc` / `662c1ef` | 2500238 | <https://youtu.be/jEHOhwIJ0F8> | Line following: straight, U-bend and S-bend |
| 2026-11-20 | System | SYS-04 | `662c1ef` (left run); `af11df6` or `1d088b2` (right run, filmed 2026-11-11) | 2500238 | <https://youtu.be/KhjVVtTxKeM> | Obstacle stop and bypass: left 0:00-0:21, right 0:21-0:49 |
| 2026-11-20 | System | SYS-06 | `662c1ef` | 2500238 | <https://youtu.be/gjNppsurhxw> | Turnover stops the robot with fault 1 |
| 2026-11-24 | S1 | S1-U07 | `f8b68dd` + uncommitted S2 low-speed kick | 2500238 | <https://youtu.be/qj4SRrp7bfA> | Robot reconnects after a broker restart |
| 2026-11-24 | System | SYS-08 | `f8b68dd` + uncommitted S2 low-speed kick | 2500238 | <https://youtu.be/9uLjh_gtLUs> | Access point off and on: robot offline, then back online |
| 2026-11-25 | S4 | S4-U36 | `98c9a60` | 2500238 | <https://youtu.be/cTbNUiUAZ10> | Pitch and roll follow the tilt on the hump ramp |
| 2026-11-25 | System | SYS-05 | `ac5485d` | 2500238 | <https://youtu.be/qSe3YlFr-ys> | Hump crossing; hump events on the dashboard |
