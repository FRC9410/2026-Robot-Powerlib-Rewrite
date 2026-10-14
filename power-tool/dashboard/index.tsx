import { DrivePanel, type DrivePanelData } from "../src/features/drive/DrivePanel";
import { formatTime } from "../src/features/drive/driveModel";
import type { GeneratedSubsystem } from "../src/features/subsystems/types";
import { gameReadings } from "./gameModel";
import "./dashboard.css";

const metric = (value: number | undefined, places = 1) => value === undefined ? "—" : value.toFixed(places);

function Check({ label, ok }: { label: string; ok?: boolean }) {
  return <div className="game2026-check"><span>{label}</span>
    <span className={ok === undefined ? "unknown" : ok ? "good" : "waiting"}>
      <i aria-hidden="true" />{ok === undefined ? "—" : ok ? "YES" : "WAIT"}
    </span>
  </div>;
}

function hubCard(data: DrivePanelData) {
  const game = gameReadings(data);
  const alliance = data.drive.text("alliance");
  const bothActive = game.hubActive === true && ["Autonomous", "Transition", "End game"].includes(game.phase ?? "");
  const activeHub = game.hubActive === undefined ? undefined : bothActive ? "both"
    : alliance === "Blue" ? (game.hubActive ? "blue" : "red")
    : alliance === "Red" ? (game.hubActive ? "red" : "blue") : undefined;
  const activeLabel = activeHub === "both" ? "Both hubs"
    : activeHub === "blue" ? "Blue hub" : activeHub === "red" ? "Red hub" : "";
  return <section className={`drive-top-metric game2026-hub ${activeHub ? `${activeHub}-active` : "unknown"}`}
    aria-label="Hub timer" title="Estimated time remaining in the current match phase">
    <span className="drive-top-metric-label">{game.hubActive === undefined ? "HUB UNKNOWN" : game.hubActive ? "HUB ACTIVE" : "HUB INACTIVE"}</span>
    <b>{formatTime(game.hubTime)}</b>
    <small>{game.phase || "Waiting for robot"}{activeLabel ? ` · ${activeLabel}` : ""} · estimated</small>
  </section>;
}

function shooterChecklist(data: DrivePanelData) {
  const game = gameReadings(data);
  const status = !data.drive.live ? "Waiting for live robot data"
    : game.requested === undefined ? "Waiting for shooting telemetry"
    : !game.requested ? "No shot requested"
    : game.hubActive === undefined ? "Hub status unknown"
    : game.feedReady && !game.hubActive ? "Mechanisms ready · hub inactive"
    : game.status || "Waiting for shot";
  return <section className="game2026-shot" aria-label="Shooter checklist">
    <div className="game2026-shot-heading"><h2>Shot ready</h2>
      <span className={game.ready === undefined ? "unknown" : game.ready ? "good" : "waiting"}>
        <i aria-hidden="true" />{game.ready === undefined ? "UNKNOWN" : game.ready ? "READY" : game.requested ? "WAIT" : "IDLE"}
      </span>
    </div>
    <Check label="Flywheel at speed" ok={game.velocity} />
    <Check label="Hood in position" ok={game.hood} />
    <Check label={`Aligned${game.headingError !== undefined ? ` · ${metric(game.headingError)}°` : ""}`} ok={game.aligned} />
    <Check label="Calibrated range" ok={game.range} />
    <Check label="Our hub active" ok={game.hubActive} />
    <div className="game2026-check"><span>Target</span><span>{game.distance === undefined ? "—" : `HUB · ${metric(game.distance, 2)} m`}</span></div>
    <div className="game2026-check"><span>Vision</span><span>{game.vision === undefined ? "—" : game.vision ? "Fresh accepted frame" : "No accepted frame"}</span></div>
    <p className="game2026-shot-status" role="status">{status}</p>
  </section>;
}

export function DashboardPanel({ subsystems = [] }: { subsystems?: GeneratedSubsystem[] }) {
  return <div className="game2026-dashboard">
    <DrivePanel subsystems={subsystems} renderTopCards={hubCard} renderSidebar={shooterChecklist} />
  </div>;
}
