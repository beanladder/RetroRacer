# AI Racing System

AI drivers for the procedurally generated tracks. Every car plans its own braking points from its
own grip, follows the racing line with pure pursuit steering, and sees the whole field — player
included — in racing line coordinates.

## Files

| File | Role |
| --- | --- |
| `AITrackData.cs` | World space geometry of a track's racing line: arc length, heading, side normals, signed curvature. Built once per track and shared by every car. |
| `AISpeedProfile.cs` | Per-car target speed at every point on the line, from the grip circle plus a backward braking pass and a forward traction pass. |
| `AIVehicleController.cs` | The driver. Steering, throttle, brake, handbrake, traffic, overtaking, nitro and recovery. |
| `RaceParticipant.cs` | One car in the race, AI or player, in racing line coordinates. |
| `AIRaceManager.cs` | Spawns the grid, runs laps and positions, keeps every driver's view of the field current. |
| `CarTriggerHandler.cs` | Routes checkpoint, lap and recovery events to the manager. |
| `AIDriverTelemetry.cs` | The decision snapshot the gizmos, the runtime sensor lines and the HUD all read. |
| `AIDebugHud.cs` | On-screen readout of the whole field, and the hotkeys for gizmos, sensor lines and the spectator camera. |
| `AISpectatorCamera.cs` | Free camera for watching AI cars: orbit or free fly, sharing the HUD's focused car. |

## How a lap is driven

**Speed.** `AISpeedProfile` starts from `v = sqrt(a / k)` at every point — the fastest the car can
hold that curvature. That alone only says how fast the apex may be taken, so two relaxation passes
run around the loop: a backward pass caps each point at the speed the car can still brake down from
before the next one, and a forward pass caps it at the speed it could have reached accelerating out
of the previous one. The braking points fall out of that automatically. Change
`corneringAcceleration` and every corner on every track re-plans.

**Steering.** Pure pursuit picks an aim point along the line, at a distance that scales with speed,
and solves for the steering arc through it. The result is mapped back through the vehicle's own
`MaxTurnAngle` and `turnCurve`, so a commanded 0.4 really is 0.4 of the lock the physics will give.
A capped cross-track term holds the line through long corners.

**Braking.** Through *negative throttle*, not the handbrake. In this vehicle model `handbrakeInput`
both slows the car and drops rear grip to `driftFactor`, so braking with it means breaking traction
every time. Negative throttle is roughly three times stronger here and keeps the car pointed where
it is going. The handbrake is left for hairpins and recovery.

**Traffic.** Every car is projected onto the racing line once per physics step by the manager, so
each driver reads its rivals as "40 metres up the road and 3 metres to my left". That answers both
whether a car is in the way and which side has room, which world space angles never did.

**Learning.** Each car watches its own sideways-to-forward velocity ratio through corners. Slide too
much and it lowers its assumed grip and re-plans; carry margin and it gradually leans harder. So a
car that is over-driving a track corrects itself instead of repeating the mistake every lap.

## Tuning

`skillLevel` and `aggressiveness` are the two dials, and the manager assigns them per car from its
own min/max ranges. Everything else derives from them:

- **skill** → grip used, lookahead distance, reaction time (0.30s down to 0.06s), how cleanly the
  line is held, and reaction at the start lights.
- **aggression** → how soon a pass is attempted, how little room is accepted, how hard the driver
  defends, and willingness to use nitro.

For outright pace, `corneringAcceleration` is by far the biggest lever. On the shipped tracks
(≈5.5–6.2 km laps, median corner radius ≈270 units) it maps roughly to:

| `corneringAcceleration` | Slowest corner | Median | Lap time |
| --- | --- | --- | --- |
| 12 | ~14 m/s | ~47 m/s | ~128 s |
| 18 (default) | ~17 m/s | ~54 m/s | ~111 s |
| 25 | ~20 m/s | ~59 m/s | ~100 s |

## Seeing what a driver is thinking

Two tools, both off by default.

**Gizmos, per car.** `AIVehicleController.debugLayers` is a flag mask; enable Gizmos in the Scene or
Game view to see them. Because AI cars are spawned at runtime you cannot set this in the inspector
beforehand — select the car during play, or press **F3** with the HUD focused on it.

| Layer | Draws |
| --- | --- |
| `SpeedProfile` | The plan ahead as a ribbon, red slow to green fast, with ribs whose height is the target speed. An amber sphere marks the **lift point** — the first place the car is already going too fast for — and a red sphere marks the slowest point it is planning around. A white rib at the car shows current speed on the same scale, so you can read the two off each other. |
| `PlannedLine` | White is the racing line, cyan is the offset line the driver has chosen, magenta is the offset itself, and yellow is the cross-track error it still has to steer out. A faint bar shows how far the offset is allowed to go. |
| `Steering` | The pure pursuit aim point, and the arc the car will *actually* follow at its current steering angle including the speed-dependent `turnCurve`. When the arc falls outside the cyan line, that is understeer, drawn. |
| `Inputs` | Throttle, brake, handbrake, nitro and speed as gauges above the roof, with a yellow tick on the speed gauge at the target, plus steering as a needle. |
| `Traffic` | A line to every rival being weighed up — red for close ahead, fading to grey at the edge of awareness, amber for alongside. The car it is stuck behind gets a magenta ring and a patience bar; the side chosen for a pass gets an arrow. |
| `Readout` | A text block above the roof: state, speed vs target, inputs, line error, grip estimate, reaction time and what it is doing about traffic. Editor only. |

**HUD, whole field.** Add `AIDebugHud` next to the `AIRaceManager`. It lists every car — player
included — with state, speed against target, pedal positions and learned grip, plus a detail panel
for one focused car. Hotkeys read `Keyboard.current`, since this project runs the new Input System
exclusively and the legacy `Input` class would throw.

| Key | Does |
| --- | --- |
| **F1** | Hide/show the HUD |
| **F2** | Step the focused car — shared with the sensor lines and the spectator camera |
| **F3** | Toggle every gizmo layer on the focused car (Scene view, or Game view with its Gizmos toggle on) |
| **F4** | Toggle the Game-view sensor and decision lines on the focused car |
| **F5** | Cycle the spectator camera: off &#8594; orbit &#8594; free fly |

**Sensor and decision lines, in the Game view.** Gizmos only draw in the Scene view, or in the Game
view with its own Gizmos toggle switched on — easy to miss, and gizmos never draw at all in a build.
**F4** instead draws real `LineRenderer` geometry on the focused car, visible in the Game view (and
in a build) with nothing else to enable:

- A line from the car to its steering aim point, coloured by <b>`AIDriverTelemetry.StateColor`</b> —
  the decision itself, at a glance: green cruising, amber lifting, red braking, magenta overtaking,
  blue defending, orange recovering.
- A curving <b>blue</b> lane either side of the car showing the actual awareness zone
  (`sideClearance` wide, `awarenessDistance` long), sampled along the track so it bends through
  corners instead of cutting across them.
- A link to every rival within that zone, coloured by the same classification the traffic logic
  itself uses: <b>red</b> for the car being avoided or lined up to pass, <b>magenta</b> once a pass
  is committed, <b>amber</b> for a car alongside, dim blue for everything else being tracked.

Only the focused car draws, both to keep a nine-car grid readable and because it is the same car the
spectator camera orbits.

**Spectator camera.** Add `AISpectatorCamera` next to `AIRaceManager` (it needs `AIDebugHud` there
too, since F5 is read by the HUD). It does not spawn a second camera: pressing F5 disables the
`CinemachineBrain` on `Camera.main` and drives that
same camera directly, then re-enables the brain on the way back to Off so Cinemachine resumes from
whichever vcam has priority. Two modes:

- **Orbit** circles the car `AIRaceManager` currently has focused — the same focus F2 steps. Hold
  the right mouse button to look around, scroll to zoom.
- **Free fly** detaches entirely. Hold the right mouse button to look, WASD to move,
  Space/E up, Q/Ctrl down, Shift to go faster — the Scene view's own fly-camera convention, chosen
  so it needs no separate explanation.

The player's own input keeps flowing to their car underneath while spectating; this is a viewing
tool, not a pause.

Turn on `verboseLogging` on the manager for grid, lap and recovery messages. It is off by default
because the old system logged from `Update` on every car.

### Reading the picture

- **Braking too late?** Watch the amber lift sphere. If it appears only as the corner arrives, the
  profile thinks the car has more grip than it does — lower `corneringAcceleration`, or check
  whether the learned `grip` figure in the HUD has drifted down on its own.
- **Weaving?** Watch the yellow cross-track line. If it oscillates side to side, either
  `cornerTrackingGain` is too high or the lookahead is too short for the speed.
- **Missing passes?** Watch the patience bar over the blocked car. If it never fills, the driver is
  not judging itself quicker; if it fills and nothing happens, both sides failed the clearance test.

## Setup

1. The track needs a `TrackGenerator` with **Generate Racing Line** enabled, or a saved track prefab
   (the racing line is serialised with it).
2. Put `AIRaceManager` on any object in the scene and assign the AI vehicle prefabs, the player
   prefab and the liveries. Vehicle prefabs need a `SimcadeVehicleController`.
3. Optionally add `AIDebugHud` and `AISpectatorCamera` on the same object, for the on-screen field
   readout and the F1–F5 hotkeys described below.
4. Press play. The manager waits for the racing line, forms the grid, runs a countdown and releases
   the cars. `AIVehicleController` and `CarTriggerHandler` are added to the spawned cars
   automatically — they are not authored on the prefabs.

## Notes

- Recovery is handled in two stages. A car that is stuck reverses itself out; one that falls out of
  the world or ends up on its roof is placed back on the racing line where it left it, facing the
  right way, with its velocity cleared.
- Lap progress is `lap + fraction of a lap`, measured from the start/finish line, and is computed
  the same way for the AI and the player, so running order is directly comparable.
- AI cars get a controlled `driftFactor`; the car prefabs ship a negative one, which would turn any
  handbrake input into a spin.
