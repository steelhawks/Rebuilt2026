# PoseLink — RIO ⇄ Orange Pi vision/pose-estimation link

The Orange Pi now owns vision + pose estimation. It runs PhotonVision, AprilTag
whitelisting/rejection, per-camera stddev weighting, and a **GTSAM iSAM2** factor
graph fusing wheel odometry, PMW3901 optical flow, and AprilTag observations.
The roboRIO is a thin client: it ships odometry to the Pi and consumes a fused
pose, keeping the rest of `RobotState` (SOTM, aim/shift state, boundary triggers,
object detection) unchanged.

This package is the **RIO side** only. The Pi service (Java + GTSAM via JNI) is a
separate deployable; it shares the wire contract in
[`src/main/proto/vision_link.proto`](../../../../../../proto/vision_link.proto).

## Message schema (Protobuf over UDP)

Both ends are Java and link the same generated classes (`org.steelhawks.proto`).
Every timestamp is a **sample time** (FPGA/match seconds), never a send time —
GTSAM factor-graph correctness depends on this.

### RIO → Pi: `RobotOdomInputs` (~50 Hz, RIO_RX≠send rate)
| field | notes |
|-------|-------|
| `seqnum` | increments per packet; Pi detects drops/reordering, never fuses an older one |
| `timestamp` | odometry sample time |
| `wheel_positions[4]` | distance + steer angle per module (FL, FR, BL, BR) |
| `gyro_angle_radians` | continuous gyro yaw |
| `chassis_speeds` | motion prior between tag observations |
| `is_on_bump` | Pi scales tag trust down while crossing a bump |
| `alliance` | drives tag whitelisting on the Pi |
| `config_hash` | RIO's field-layout/alliance-tag hash; Pi flags a mismatch |
| `reset_seqnum` + `reset_{x,y,theta}` | pose-reset command; Pi applies each new seqnum once (idempotent) |

### Pi → RIO: `FusedPoseOutput` (every cycle)
| field | notes |
|-------|-------|
| `seqnum`, `timestamp` | RIO rejects any output not strictly newer (drop/reorder guard) |
| `x, y, theta` | fused pose |
| `quality_score` + `cov_{xx,yy,theta}` | from GTSAM marginal covariance |
| `config_hash` | checked against RIO's; logged loudly on mismatch, pose dropped |
| `solve_latency_ms` | GTSAM solve time, diagnostics |
| `ack_reset_seqnum` | echoes the last reset the Pi applied |

Ports/host live in `PoseLinkConstants` (`PI_HOST`, `PI_RX_PORT` 5812, `RIO_RX_PORT` 5811).
`PI_RX_PORT` must never be 5810 — photonlib starts a `TimeSyncServer` there inside
the Pi service process, and stealing it makes PhotonVision drop every frame.

`session_id` is minted per RIO boot (`LogSession`) and echoed by the Pi so the two
logs can be paired afterwards; `tools/pull_logs.py` does the pairing.

## Reliability behavior (RIO side)

- **Sequence + timestamp monotonicity** — `RobotState.applyFusedPose` refuses any
  observation not strictly newer than the last applied; `PoseLinkIOUDP` also
  counts seqnum gaps (`packetsDropped`) and out-of-order arrivals (`packetsStale`).
- **Config-hash check** — `PoseLink` compares `FusedPoseOutput.config_hash` to
  `PoseLinkConstants.CONFIG_HASH`; on mismatch it raises an `Alert`, logs
  `PoseLink/ConfigMismatch`, and **drops** the pose rather than trusting it.
- **Staleness timeout + fallback** — chosen value **`STALENESS_TIMEOUT_SEC = 0.2`
  s** (200 ms). At the 50 Hz link rate that tolerates ~10 dropped packets before
  falling back. Past it, `RobotState.getEstimatedPose()` returns a locally
  maintained **wheel-only dead-reckoning** pose (a `SwerveDriveOdometry` kept
  updating in parallel the whole match — never frozen). Recovery is automatic the
  moment fresh fused data resumes. Tune from testing; the design target was
  150–250 ms.
- **Sample-time stamping** — outgoing packets carry the odometry sample time;
  incoming poses are applied at the Pi's reported sample time.

## Logging (AdvantageKit)

- RIO logs every received `FusedPoseOutput` and link health under `PoseLink/*`
  (`PoseLinkIOInputs`, `UsingFallback`, `SecondsSinceLastRx`, `ConfigMismatch`)
  and pose state under `RobotState/PoseEstimation/*` and `RobotState/PoseLink/*`.
- The Pi should log its GTSAM internals (factor count, solve time, per-observation
  residuals) so a full match's sensor stream can be replayed against different
  factor-noise tunings offline.

## Replaying a logged match against the Pi for GTSAM tuning

Because every `RobotOdomInputs` the RIO sends is derived from logged odometry and
every `FusedPoseOutput` is logged on receipt, a match can be replayed offline:

1. Pull the match `.wpilog` from the RIO (AdvantageScope or `matchlogs/`).
2. Run the Pi service in **replay mode**: instead of a live UDP socket, feed it the
   logged `RobotOdomInputs` stream (same Protobuf messages) in timestamp order,
   plus the logged PhotonVision frames from the Pi's own log.
3. Sweep GTSAM factor noise models; compare the recomputed `FusedPoseOutput`
   against the logged one (and against AprilTag-only pose snapshots).
4. AdvantageScope overlays `RobotState/PoseEstimation/PoseEstimation` (fused),
   `.../Odometry` (wheel-only fallback), and `RobotState/PoseLink/AppliedPose` to
   eyeball drift and fallback transitions.

Since the RIO side is unchanged by tuning, you only redeploy the Pi jar (see the
Pi service's `systemd` unit + SSH deploy script — separate deliverable).
```
