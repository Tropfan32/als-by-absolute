# AACS (Autonomous Adaptive Control System)

FTC Robot Controller **quickstart** for [Pedro Pathing](https://pedropathing.com) plus an **Autonomous Adaptive Control System (AACS)** layer: detect collisions, rebuild a Bézier back to the goal, and keep that curve out of illegal field zones.

This repo is based on the Pedro Pathing Quickstart (FTC SDK **11.1**, DECODE 2025–2026) with team code under `TeamCode`.

Full documentation site (separate repo): [aqqusr/aacssite](https://github.com/aqqusr/aacssite) — live at [aacs.vercel.app](https://aacs.vercel.app). This repository is **robot code only** (Android Studio / FTC SDK). Brand on the site: `#1E1E1E` field, logo purple circular washes, white type.

## What you get

| Piece | Where | Role |
| --- | --- | --- |
| Pedro follower + Pinpoint | `pedroPathing/Constants.java` | Localization and path following |
| `ImpactDetector` | `adaptive/` | Flags a real hit (G-threshold + 50 ms window) |
| `AdaptivePathPlanner` | `adaptive/` | On hit: new quadratic Bézier from live pose → target |
| `FoulPreventionBox` | `adaptive/` | Geofence: field walls + Alliance/opponent AABBs |
| `AdaptiveTestOpMode` | `adaptive/` | TeleOp to exercise the full pipeline |
| Curve Drive Test | `aacs.java` | Auto: one quadratic Bézier + AACS watch / force replan |

Loop contract (do not reorder):

1. Read linear acceleration from the follower / Pinpoint  
2. `impactDetector.update(ax, ay)`  
3. `planner.update()`  
4. `follower.update()`

## Requirements

- Android Studio **Ladybug (2024.2)** or newer  
- FTC Control Hub / Driver Station  
- goBILDA **Pinpoint** on I2C, hardware map name `pinpoint`  
- Mecanum (or whatever drivetrain you already tuned in Pedro)

Official FTC docs: [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/index.html)  
Pedro docs: [pedropathing.com](https://pedropathing.com/docs/pathing/examples/constants)

## Open the project

```text
git clone https://github.com/aqqusr/aacs.git
```

In Android Studio: **Open** the cloned folder (Gradle project). Let it sync, then deploy **TeamCode** + **FtcRobotController** to the Robot Controller.

## Hardware: Pinpoint offsets

Offsets are in `TeamCode/.../pedroPathing/Constants.java`:

- Strafe pod X: **−120 mm**  
- Forward pod Y: **−150 mm**  
- Device name: `pinpoint`  
- 4-bar goBILDA pods, both encoders `FORWARD`

If tracking is mirrored or scaled wrong, retune with Pedro’s **Tuning** OpMode (`Localization` → offsets / forward / lateral / turn), then paste the new `strafePodX` / `forwardPodY` into `PinpointConstants`.

Keep the robot **still** through OpMode `init` so the Pinpoint IMU can finish calibration.

## Run the adaptive test

1. Place the robot on start pose **(36 in, 24 in, 0°)** in Pedro coordinates (origin at a field corner, axes `[0, 144]`).  
2. Driver Station → TeleOp → **Adaptive Pipeline Test** (`Adaptive` group).  
3. Press PLAY. Gamepad 1:

| Button | Action |
| --- | --- |
| **A** | Set goal to **Target Alpha** `(108, 36, 0°)` and follow a straight path there |
| **B** | Set recovery goal to **Target Beta** `(108, 108, 90°)` (no new path by itself) |
| **X** | `forceReplan()` — fake a recovery without crashing the robot |

Telemetry shows pose, accel (m/s² and g), impact latch, replan count, and cooldown.

### Curve Drive Test (Autonomous)

Driver Station → Autonomous → **Curve Drive Test**. Robot start ~`(24, 24)` in, heading 0°. PLAY follows a quadratic `(24,24) → (96,24) → (96,72)`. Gamepad 1 **A** restarts the curve from the live pose; **X** calls `forceReplan()` to the end pose.

To bump a robot into a real replan: start Alpha with **A**, then shove the chassis. After ~50 ms above **1.5 g** horizontal, the planner injects a quadratic Bézier to the current target (with a **300 ms** cooldown so it does not thrash).

## Use it in your own Auto

```java
Follower follower = Constants.createFollower(hardwareMap);
follower.setStartingPose(startPose);

ImpactDetector impactDetector = new ImpactDetector(); // 1.5 g, 50 ms
FoulPreventionBox geofence = new FoulPreventionBox();
geofence.setFieldBounds(0, 0, 144, 144);
geofence.setClearanceInches(6);
geofence.addAabb("opponent_wing", 0, 0, 24, 48);
// add your season’s illegal boxes here

AdaptivePathPlanner planner = new AdaptivePathPlanner(follower, impactDetector);
planner.setFoulPreventionBox(geofence);
planner.setTargetPose(scorePose);

// loop():
impactDetector.update(axMetersPerSec2, ayMetersPerSec2);
planner.update();
follower.update();
```

Call `planner.setTargetPose(...)` whenever Auto changes goals (intake → score → park). Acceleration must be **m/s²**, not g; Pedro’s `follower.getAcceleration()` is inches/s² — multiply by `0.0254`.

## Tuning knobs

**ImpactDetector**

- `setThresholdG(1.5)` — raise if drivetrain vibration false-triggers  
- `setSustainWindowMs(50)` — longer window = fewer false hits, slower reaction  

**AdaptivePathPlanner**

- `setCooldownMs(300)` — 200–500 ms typical  
- `setControlPointFraction(0.35)` — how far `P1` sits along velocity for a smooth tangent  
- `setEnabled(false)` — consume impacts but do not replan  

**FoulPreventionBox**

- Sample test boxes (opponent wing + center “submersible”) live in `AdaptiveTestOpMode.buildGeofence()`. **Replace them** with current-game illegal zones so you do not foul.  
- `setClearanceInches(6)` — robot half-width / tile margin  
- If a recovery curve still clips a box after `P1` is pushed out, the planner **skips** that replan instead of injecting an illegal path  

## Package map

```text
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
  aacs.java                 // Curve Drive Test OpMode
  adaptive/
    ImpactDetector.java
    AdaptivePathPlanner.java
    FoulPreventionBox.java
    Point.java
    AdaptiveTestOpMode.java
  pedroPathing/
    Constants.java      // Follower + Pinpoint
    Tuning.java         // Pedro’s built-in tuners
```

## FTC SDK notice

Robot Controller app and samples come from FIRST’s public FTC SDK for DECODE. SDK changelog and sample OpModes remain in the original project layout (`FtcRobotController/.../samples`). You do not need that tree to run the adaptive pipeline, but you do need a legal, up-to-date Robot Controller on the Hub.
