import { useEffect, useState } from "react";
import { DrivePanel, type DrivePanelData } from "../src/features/drive/DrivePanel";
import { formatTime } from "../src/features/drive/driveModel";
import { useNetworkTables } from "../src/features/networktables/NetworkTablesContext";
import type { GeneratedSubsystem } from "../src/features/subsystems/types";
import { gameReadings } from "./gameModel";
import "./dashboard.css";

const metric = (value: number | undefined, places = 1) => value === undefined ? "—" : value.toFixed(places);
// Read every robot loop by SotmSwitch, independent of tuning mode.
const SOTM_REQUEST_TOPIC = "/PowerLib/Shooting/SOTMRequestedEnabled";
const SOTM_CONFIRM_TIMEOUT_MS = 3000;

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

/** The button shows the robot's reported state; a click requests the opposite until the robot confirms. */
function SotmToggle({ enabled, moving, live }: { enabled?: boolean; moving?: boolean; live: boolean }) {
  const { status, clientRef } = useNetworkTables();
  const [pending, setPending] = useState<{ value: boolean; at: number } | null>(null);
  const [error, setError] = useState("");
  useEffect(() => {
    if (!pending) return;
    if (enabled === pending.value) { setPending(null); return; }
    const timer = setTimeout(() => { setPending(null); setError("Robot did not confirm the change"); },
      Math.max(0, pending.at + SOTM_CONFIRM_TIMEOUT_MS - performance.now()));
    return () => clearTimeout(timer);
  }, [pending, enabled]);
  const usable = live && status === "connected" && enabled !== undefined && !pending;

  async function toggle() {
    if (!usable) return;
    const value = !enabled;
    setError("");
    setPending({ value, at: performance.now() });
    try { await clientRef.current.publish(SOTM_REQUEST_TOPIC, "boolean", value); }
    catch (cause) { setPending(null); setError(cause instanceof Error ? cause.message : "Switch failed"); }
  }

  return <>
    <div className="game2026-check"><span>Shoot on move</span>
      <button type="button" className={`game2026-toggle ${enabled === undefined ? "unknown" : enabled ? "on" : "off"}`}
        disabled={!usable} aria-pressed={enabled} onClick={() => void toggle()}
        title={enabled === undefined ? "Waiting for the robot" : enabled ? "Turn shoot on the move off" : "Turn shoot on the move on"}>
        {pending ? "SWITCHING…" : enabled === undefined ? "—" : !enabled ? "OFF" : moving ? "ON · MOVING" : "ON"}
      </button>
    </div>
    {error && <p className="game2026-toggle-error" role="alert">{error}</p>}
  </>;
}

function ShooterChecklist({ data }: { data: DrivePanelData }) {
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
    <SotmToggle enabled={game.sotmEnabled} moving={game.moving} live={data.drive.live} />
    <div className="game2026-check"><span>SOTM confidence</span>
      <span>{game.sotmValid && game.confidence !== undefined ? `${Math.round(game.confidence)}%` : "—"}</span></div>
    <div className="game2026-check"><span>Lead</span>
      <span>{game.lead === undefined ? "—" : `${metric(game.lead)}°${game.flightTime !== undefined ? ` · ${metric(game.flightTime, 2)} s flight` : ""}`}</span></div>
    <div className="game2026-check"><span>Target</span><span>{game.distance === undefined ? "—"
      : `HUB · ${metric(game.distance, 2)} m${game.moving && game.aimDistance !== undefined ? ` · aim ${metric(game.aimDistance, 2)} m` : ""}`}</span></div>
    <div className="game2026-check"><span>Vision</span><span>{game.vision === undefined ? "—" : game.vision ? "Fresh accepted frame" : "No accepted frame"}</span></div>
    <p className="game2026-shot-status" role="status">{status}</p>
  </section>;
}

export function DashboardPanel({ subsystems = [] }: { subsystems?: GeneratedSubsystem[] }) {
  return <div className="game2026-dashboard">
    <DrivePanel subsystems={subsystems} renderTopCards={hubCard} renderSidebar={data => <ShooterChecklist data={data} />} />
  </div>;
}
