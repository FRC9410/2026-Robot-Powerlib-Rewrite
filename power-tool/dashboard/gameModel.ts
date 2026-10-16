import type { DrivePanelData } from "../src/features/drive/DrivePanel";

// The display clock ticks every 100 ms but frames are stamped on arrival, so a fresh frame
// can be up to one tick "ahead" of `now`. Rejecting that made the card blink to UNKNOWN.
const CLOCK_LAG_TOLERANCE_MS = 250;

export function gameReadings({ topics, drive, now }: DrivePanelData) {
  const entries = new Map(topics.map(topic => [topic.name, topic]));
  const get = (owner: string, key: string) => {
    const topic = entries.get(`/PowerLib/Subsystems/${owner}/Data/${key}`);
    return drive.live && topic?.receivedAt !== undefined && topic.receivedAt - now <= CLOCK_LAG_TOLERANCE_MS
      && now - topic.receivedAt < 1200 ? topic.value : undefined;
  };
  const bool = (owner: string, key: string) => typeof get(owner, key) === "boolean" ? get(owner, key) as boolean : undefined;
  const number = (owner: string, key: string) => typeof get(owner, key) === "number" && Number.isFinite(get(owner, key))
    ? get(owner, key) as number : undefined;
  const text = (owner: string, key: string) => typeof get(owner, key) === "string" ? get(owner, key) as string : undefined;
  const requested = bool("Shooting", "ShotRequested");
  const hubActive = bool("Game2026", "HubActive");
  const feedReady = bool("Shooting", "FeedReady");
  return {
    requested, hubActive, feedReady,
    ready: requested === false ? false : requested === undefined || hubActive === undefined || feedReady === undefined
      ? undefined : requested && hubActive && feedReady,
    phase: text("Game2026", "HubPhase"),
    hubTime: number("Game2026", "HubTimeSeconds"),
    velocity: requested ? bool("Shooting", "VelocityReady") : undefined,
    hood: requested ? bool("Shooting", "HoodReady") : undefined,
    aligned: requested ? bool("Shooting", "Aligned") : undefined,
    range: requested ? bool("Shooting", "CalibratedRange") : undefined,
    headingError: requested ? number("Shooting", "HeadingErrorDegrees") : undefined,
    distance: number("Shooting", "Distance"),
    sotmEnabled: bool("Shooting", "SOTMEnabled"),
    moving: requested ? bool("Shooting", "MotionCompensated") : undefined,
    sotmValid: requested ? bool("Shooting", "SOTMValid") : undefined,
    confidence: requested ? number("Shooting", "SOTMConfidence") : undefined,
    lead: requested ? number("Shooting", "LeadDegrees") : undefined,
    flightTime: requested ? number("Shooting", "TimeOfFlightSeconds") : undefined,
    aimDistance: requested ? number("Shooting", "EffectiveDistanceMeters") : undefined,
    vision: bool("Shooting", "VisionAccepted"),
    status: text("Shooting", "Status")
  };
}
