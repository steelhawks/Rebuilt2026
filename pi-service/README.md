# pi-service — Orange Pi vision/pose-estimation service

Headless AdvantageKit `LoggedRobot` (Idun pattern) that will own PhotonVision,
tag whitelisting/rejection, stddev weighting, and GTSAM iSAM2 pose fusion, and
speak the `vision_link.proto` contract (in `:common`) to the RIO over UDP.

See `src/main/java/org/steelhawks/subsystems/poselink/README.md` (RIO side) for
the wire schema and the full design.

## Build layout
- **Java** (`src/main/java`) — the `LoggedRobot`, IO layers, Java rejection/stddev
  port, JNI facade. Built by Gradle (`:pi-service`), depends on `:common`.
- **Native** (`cpp/`) — `libposelink_gtsam.so`, the narrow hand-JNI shim over
  GTSAM. Built by CMake, on the Pi.

## Slice status
- **Slice 1 (done):** scaffolding + bring-up smoke test (akit-on-arm64 +
  JNI load).
- **Slice 2 (this):** the real estimator. `cpp/pose_graph.{h,cpp}` is a SE(2)
  `IncrementalFixedLagSmoother` — odom `BetweenFactor` (gyro in the rotation),
  each accepted tag an absolute `PriorFactor`, with splice-or-attach. Splicing
  is done in a **staging window** (`stageWindow_`, ~150 ms): recent odom edges
  are held out of the smoother so a late tag splits an edge still in our own
  buffer, and the reported pose is the last smoothed frontier forward-composed
  over the staged deltas (which latency-compensates the output). No GTSAM factor
  is ever removed. **Requires GTSAM on the build host — validated on the Pi**
  (not compilable on a stock dev box without GTSAM installed).
- **Slice 3 (this):** the Java fusion loop. `link/RioLink` (UDP: decode
  `RobotOdomInputs`, send `FusedPoseOutput`), `vision/PhotonVisionIOReal`
  (photonlib multi-tag + single-tag decode), `vision/VisionFilter` (the
  `Vision.java` whitelist/rejection/stddev port), `vision/TimeSync` (RIO clock
  via NT offset). `PiRobot.robotPeriodic` runs it at 50 Hz: odom -> BetweenFactor,
  filtered tags -> PriorFactor, solve, emit (withheld until anchored). Compiles
  against real WPILib/photonlib; **runs on the Pi** (needs the RIO + PhotonVision).
- **Slice 4 (next):** deploy (systemd + `deploy_pi.sh`).

### Config to fill in before running on the robot
`PiVisionConstants`: the full camera list + extrinsics (only 2 ported so far),
`TEAM_NUMBER`, and `RIO_HOST`. (No drivetrain geometry needed - the RIO sends its
cumulative odometry pose and the Pi differences consecutive samples.)

## Prerequisites on the Pi (aarch64 / linuxarm64)
- JDK **17** aarch64 (Liberica/Temurin) — matches the toolchain.
- `cmake` ≥ 3.16, `build-essential` (gcc/g++).
- (Slice 2) GTSAM built from source, pinned, `-DGTSAM_USE_TBB=OFF`, static.

## Build & run the smoke test

```bash
# 1. Native shim -> libposelink_gtsam.so
cd pi-service/cpp
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build            # produces build/libposelink_gtsam.so

# (optional) run the estimator sanity test
cmake --build build --target pose_graph_test && ./build/pose_graph_test

# 2. Service fat jar
cd ../..
./gradlew :pi-service:jar       # build/libs/pi-service-*.jar

# 3. Run it, pointing java.library.path at the shim
java -Djava.library.path=pi-service/cpp/build \
     -jar pi-service/build/libs/pi-service*.jar
```

**Expected output (smoke test passing):**
```
[poselink] native: poselink_gtsam 1.0 (GTSAM 4.x.x)
[poselink] round-trip: x=1.5xx y=2.000 theta=0.000 status=0 nodes=... factors=...
```
(x advances ~0.5 m in +x from the reset pose; exact node/factor counts depend on
the staging window.) Plus a `.wpilog` from `WPILOGWriter`. If those lines print,
the GTSAM solver ran and **akit-on-arm64 + JNI both work**.

### If `Logger.start()` fails to load akit native on arm64
That's the documented fork from Q9: keep the same IO-pure structure but swap the
logging backend from AdvantageKit `Logger` to raw WPILib `DataLog` + a thin
custom replay harness. Nothing else in the pipeline changes.

## Local dev (x86-64 / macOS)
`./gradlew :pi-service:compileJava` builds the Java. The native shim also builds
with the same CMake (`libposelink_gtsam.dylib` on macOS); set `java.library.path`
to `cpp/build` when running locally.

## Deploy (slice 4)
`deploy_pi.sh` will `rsync` the jar + `libposelink_gtsam.so` to the Pi and
restart a `Restart=always` systemd unit ordered after network + PhotonVision.
The Pi is **not** a GradleRIO deploy target — SSH only.
