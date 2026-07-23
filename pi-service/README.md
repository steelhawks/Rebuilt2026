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
**Slice 1 (done): scaffolding + bring-up smoke test.** Proves the two riskiest
unknowns before any GTSAM logic:
1. AdvantageKit's **linuxarm64** native actually runs on the Pi (`Logger.start()`).
2. **JNI** loads `libposelink_gtsam.so` and round-trips.

The native side (`cpp/poselink_gtsam.cpp`) is a **stub** — correct JNI surface,
no factor graph yet. Slice 2 replaces its guts with a
`gtsam::IncrementalFixedLagSmoother` (SE(2); odom `BetweenFactor` with gyro
rotation; vision `PriorFactor` via splice-or-attach) behind the **same**
signatures.

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

# 2. Service fat jar
cd ../..
./gradlew :pi-service:jar       # build/libs/pi-service-*.jar

# 3. Run it, pointing java.library.path at the shim
java -Djava.library.path=pi-service/cpp/build \
     -jar pi-service/build/libs/pi-service*.jar
```

**Expected output (smoke test passing):**
```
[poselink] native: poselink_gtsam stub 0.1 (no GTSAM linked yet)
[poselink] round-trip: x=1.100 y=2.000 theta=0.000 status=0 nodes=2
```
plus a `.wpilog` written by `WPILOGWriter`. If those two lines print and the log
appears, **akit-on-arm64 and JNI both work** and slice 2 can proceed on the
LoggedRobot backend (design Q9).

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
