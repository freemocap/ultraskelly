"""The robot's control panel: an always-on web service that turns the bot on and off.

The bot (ultraskelly-bot.service) serves the live dashboard on port 8080 only while it runs.
This service runs all the time (ultraskelly-control.service, port 80) and serves the same page
with a control column: start, stop and restart the bot, switch following on and off, stop every
motor, and see whether the bot runs, since when, its last error and what works on the robot.
The page reads the live views straight from the bot's /stream; motor tests pass through here.

It loads none of the bot's code, so it keeps working when the bot's code does not. On the Pi it
manages the bot through systemd, which polkit lets it do for that one unit and nothing else
(scripts/systemd/ultraskelly-control.rules); `--local-bot` runs a local stand-in instead, for
development without systemd. See docs/dashboard.md, "Control panel".
"""
