import assert from "node:assert/strict";
import { build } from "esbuild";
import { createRequire } from "node:module";
import { fileURLToPath } from "node:url";

const result = await build({ entryPoints: [fileURLToPath(new URL("./gameModel.ts", import.meta.url))],
  bundle: true, write: false, format: "cjs", platform: "node" });
const module = { exports: {} };
new Function("require", "module", "exports", result.outputFiles[0].text)(createRequire(import.meta.url), module, module.exports);
const { gameReadings } = module.exports;
const now = 100;
const topic = (owner, key, value, receivedAt = now) => ({
  name: `/PowerLib/Subsystems/${owner}/Data/${key}`, type: typeof value, value, receivedAt
});
const topics = [
  topic("Shooting", "ShotRequested", true), topic("Shooting", "FeedReady", true),
  topic("Shooting", "VelocityReady", true), topic("Shooting", "HoodReady", true),
  topic("Shooting", "Aligned", true), topic("Shooting", "CalibratedRange", true),
  topic("Shooting", "Distance", 3.56), topic("Shooting", "HeadingErrorDegrees", -4),
  topic("Game2026", "HubActive", true), topic("Game2026", "HubTimeSeconds", 8)
];
const read = (values = topics, live = true, at = now) => gameReadings({ topics: values, drive: { live }, now: at });
assert.equal(read().ready, true);
assert.equal(read().distance, 3.56);
assert.equal(read(topics.map(t => t.name.endsWith("/HubActive") ? { ...t, value: false } : t)).ready, false);
assert.equal(read(topics.map(t => t.name.endsWith("/HubActive") ? { ...t, value: null } : t)).ready, undefined);
const idle = read(topics.map(t => t.name.endsWith("/ShotRequested") ? { ...t, value: false } : t));
assert.equal(idle.ready, false);
assert.equal(idle.velocity, undefined);
assert.equal(read([], true).ready, undefined);
assert.equal(read(topics, false).distance, undefined);
assert.equal(read(topics, true, now + 1200).ready, undefined);
assert.equal(read(topics, true, now - 1).ready, undefined);
assert.equal(read(topics.map(t => t.name.endsWith("/Distance") ? { ...t, value: NaN } : t)).distance, undefined);
assert.equal(read(topics.map(t => t.name.endsWith("/HubActive") ? { ...t, value: "true" } : t)).hubActive, undefined);
console.log("PASS: readiness, inactive/unknown hub, idle, missing/stale/future data and invalid values.");
