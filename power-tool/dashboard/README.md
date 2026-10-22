# 2026 robot dashboard

This project-owned dashboard extends the generic `DrivePanel` with a hub timer
in the top row and a shooter checklist below the autonomous selector. Components,
styles and assets here survive Power Tool installs and updates.

The generic field/camera views, autonomous selector, drivetrain card, mechanism
cards, setpoints and status bar are reused. The field view draws the robot over
`assets/field-2026.webp`, the 2026 REBUILT field without fuel from the WPILib
2026 Elastic install (`2026-field-no-fuel.png`, originally from
https://www.chiefdelphi.com/t/2026-rebuilt-high-resolution-field-image/510611).
It is cropped to that image's field corners (524, 94)–(3378, 1490) so its edges
are the 16.54 × 8.07 m field boundary, blue alliance on the left. Its `renderTopCards` and `renderSidebar`
callbacks share the existing 100 ms display clock and NetworkTables view.

`Game2026Telemetry` publishes the prepared shooting state's readiness flags to
`Shooting` in the existing `/PowerLib/Data` JSON frame. The checklist includes
flywheel speed, hood position, alignment, valid calibrated shot, hub activity,
shooting distance and accepted vision freshness. The **Shoot on move** button is
the robot's SOTM kill switch: it shows the robot's reported state (ON, ON · MOVING,
or OFF) and a click publishes the opposite to
`/PowerLib/Shooting/SOTMRequestedEnabled`, which the robot applies every loop
without tuning mode. It reads SWITCHING… until the robot confirms, and reports an
error after three seconds without confirmation. It is disabled while robot data is
not live. Shoot-on-the-move rows also show solver confidence, the lead angle with its
flight time, and the compensated aim distance while moving; those per-shot values
appear only while a shot is requested. Overall ready requires the
robot's mechanical readiness and a known active hub. Disabled/idle readings and
missing, invalid, stale or disconnected feedback display idle/unknown values.

`HubTimer` uses Driver Station alliance, enabled mode, match time and the FMS
first-inactive alliance message. It follows 20 seconds of autonomous, 10 seconds
of transition, four 25-second shifts and 30 seconds of endgame. The countdown is
remaining time in the current phase. It is an estimate because DS match time is
approximate. The hub timer background is blue or red for the currently active
alliance, split blue/red when both hubs are active, and neutral when unknown.
Missing game data during alternating shifts leaves hub status
unknown; both hubs are active in autonomous, transition and endgame.

Source: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html

From `power-tool/`, run `node dashboard/game-model-check.mjs` for readiness and
freshness checks, `npm run build` to rebuild, or `npm run package:app` to package.
The robot's `HubTimerTest` checks shift boundaries and missing inputs with
`./gradlew.bat test` from the robot root. Deploy the robot telemetry changes to
populate the new panels.