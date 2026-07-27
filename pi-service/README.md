# PoseLink

On-robot vision and pose estimation for the Orange Pi.

PoseLink moves AprilTag pose estimation off the roboRIO. Instead of the RIO
running a WPILib `SwerveDrivePoseEstimator`, the Pi runs a GTSAM factor-graph
estimator that fuses wheel odometry and AprilTag observations, and ships a single
fused pose back to the RIO over UDP. The RIO keeps everything else it always did
(SOTM, aiming, boundary triggers) and just reads the pose the Pi sends it.

This directory (`pi-service`) is the Pi program. The RIO side is the `PoseLink`
subsystem back in the main robot project, and the wire format both sides share
lives in the `:common` module.

## How it fits together

```
        cameras                 NetworkTables (server on RIO)
          |                      ^                 |
          v                      |  tag results    v
   [ PhotonVision ] ---- publishes ----> [ pi-service (this) ]
     (its own process, one or                 |  GTSAM fusion
      more Pis)                               |  fused pose (UDP)
                                               v
                                          [ roboRIO ]
                                         wheel odometry (UDP) --^
```

Two things run on the Pi, as separate processes:

- **PhotonVision** does the camera work and publishes AprilTag results to
  NetworkTables. It runs on its own, installed the normal PhotonVision way.
- **pi-service** (this program) subscribes to those results, runs the factor
  graph, and talks to the RIO over UDP.

The RIO sends its cumulative wheel-odometry pose each cycle; the Pi differences
consecutive poses into odometry constraints, drops in the AprilTag observations
as pose measurements, solves, and sends back `(x, y, theta)` plus a covariance.
If the Pi link goes quiet, the RIO falls back to plain wheel odometry on its own,
so a dropped Pi never freezes the robot's pose.

The estimator is a fixed-lag smoother, so it re-optimizes the last ~1.5 seconds
of trajectory every update. Late AprilTag frames (they always arrive a little
after the fact) get spliced into the graph at their real capture time rather than
snapped to the nearest odometry sample.

## Layout

```
common/                         shared between RIO and Pi
  src/main/proto/vision_link.proto     the UDP message format
  src/main/java/.../VisionLinkConfig   tag sets + config hash

pi-service/
  src/main/java/org/steelhawks/pi/
    PiRobot.java                the 50 Hz loop
    PiVisionConstants.java      cameras, thresholds, team/host config
    link/RioLink.java           UDP in/out
    vision/                     PhotonVision decode + tag filtering
    gtsam/NativePoseEstimator   JNI wrapper over the C++ shim
  cpp/
    pose_graph.{h,cpp}          the GTSAM estimator
    poselink_gtsam.cpp          JNI glue
    test_pose_graph.cpp         standalone estimator test
    CMakeLists.txt
  deploy/
    poselink.service            systemd unit
    deploy_pi.sh                build + push to the Pi
```

The C++ side owns the GTSAM math. The Java side owns everything else: reading the
cameras, tag whitelisting and rejection, the standard-deviation weighting, the
UDP link, and logging. The two meet at a small JNI surface (`create`, `reset`,
`addOdometry`, `addVisionMeasurement`, `update`, `getResult`).

## What you need

**Hardware.** An Orange Pi 5 or 5 Plus (RK3588, 8 GB or more). It's running
PhotonVision, a JVM, and GTSAM at the same time, so don't try this on a Zero or a
Pi 3. Give it a static IP on the robot network.

**OS.** 64-bit Ubuntu 22.04 for the board (the `ubuntu-rockchip` images are what
PhotonVision documents for the Orange Pi 5). It has to be arm64 — we ship arm64
native libraries. Armbian works too.

**Packages** (Pi and any dev machine that builds the native code):

```bash
sudo apt install openjdk-17-jdk cmake build-essential libboost-all-dev libeigen3-dev
```

**GTSAM** has to be built from source (see below). Everything else comes from
apt or Gradle.

## Building

There are three pieces: GTSAM (once), the native shim, and the Java jar. You
build GTSAM and the shim on whatever machine will run them (the Pi for real use,
your laptop if you just want to run the estimator test). The jar you build on a
dev machine and copy over.

### 1. GTSAM (one time)

Clone it somewhere outside this repo, build it, install it. Turn TBB off (keeps
the solver single-threaded, which makes log replays reproducible) and build the
unstable module (the fixed-lag smoother lives there).

```bash
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_TBB=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc)          # ~20-40 min on the Pi
sudo make install        # installs to /usr/local
```

Notes:

- On a recent CMake (4.x) GTSAM's own build errors out with a policy message.
  Add `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to the `cmake` line and it goes
  through.
- If you ever reconfigure and TBB won't turn off, wipe the `build` directory
  first. A stale CMake cache keeps the old setting. Check it took with
  `grep GTSAM_USE_TBB /usr/local/include/gtsam/config.h` (it should be commented
  out).
- You can delete the `~/gtsam` clone after `make install`. The shim only needs
  the installed headers and libs.

### 2. Native shim

From the repo root:

```bash
cmake -S pi-service/cpp -B pi-service/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build pi-service/cpp/build
```

That produces `libposelink_gtsam.so` (`.dylib` on a Mac). To sanity-check the
estimator itself:

```bash
cmake --build pi-service/cpp/build --target pose_graph_test
./pi-service/cpp/build/pose_graph_test
```

It should print a few `PASS` lines and `ALL PASS`. Run it after any change to
`pose_graph.cpp` — it covers reset, odometry integration, a tag pulling the
estimate, and the splice path.

### 3. Service jar

From the repo root:

```bash
./gradlew :pi-service:jar
```

This builds a self-contained jar with the arm64 WPILib native libraries baked in,
so it runs on the Pi as-is. It needs internet the first time to pull those arm64
artifacts from the WPILib maven repo. The jar lands in
`pi-service/build/libs/`.

To compile and iterate on the Java without building the full jar:

```bash
./gradlew :pi-service:compileJava
```

That doesn't touch the arm64 libraries, so it works offline.

## Deploying to the Pi

### First-time Pi setup

1. Install the packages and GTSAM above.
2. Install the systemd unit:
   ```bash
   sudo cp pi-service/deploy/poselink.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable poselink
   ```
   If your Pi login user or paths aren't `orangepi` / `/home/orangepi/poselink`,
   edit the unit first.

### Every deploy

Plain `deploy` only touches the roboRIO, so the WPILib "Deploy Robot Code" button
in VS Code still works. To deploy both halves, use `deployAll`:

```bash
./gradlew deployAll         # roboRIO + Pi
./gradlew deployPi          # Pi only
./gradlew deploy            # roboRIO only
```

IntelliJ users: save `deployAll` as a Gradle run configuration so it's one click.
Vision is required to play, so `deployAll` is what you want for anything real.

You can also run the script directly (it builds its own jar in this case):

```bash
PI_HOST=10.26.1.11 pi-service/deploy/deploy_pi.sh
```

Either way it builds the jar, copies it and the `cpp/` source to the Pi, builds
`libposelink_gtsam.so` on the Pi, and restarts the service. Override `PI_USER`,
`PI_HOST`, `DEPLOY_DIR`, or `SERVICE` with environment variables.

If the Pi is off or unreachable, `deployAll` reports the Pi step as failed (on
purpose — that's your signal the Pi didn't get updated).

### Checking it's alive

```bash
ssh orangepi@10.26.1.11 journalctl -u poselink -f
```

You'll also see it in AdvantageScope on the RIO under the `PoseLinkPi/*` keys
(status, solve time, node/factor counts), and the fused pose shows up on the RIO
side wherever the robot pose is logged. The service holds its output until the
graph has an anchor (a reset from the RIO or the first good multi-tag frame), so
until then the RIO stays on wheel odometry, which is normal at startup.

## Configuration

Everything you'll actually need to set is in `PiVisionConstants`:

- **`CAMERAS`** — the AprilTag cameras and their positions on the robot, plus a
  per-camera trust factor. These are the robot-to-camera transforms; they have to
  match how the cameras are physically mounted.
- **`TEAM_NUMBER`** — points the NetworkTables client at the RIO.
- **`RIO_HOST`** / ports — the UDP endpoints.

Rejection thresholds and the standard-deviation weighting are in there too if you
want to tune how much the fusion trusts vision.

There's no drivetrain geometry to configure. The RIO sends its odometry as a
pose, and the Pi just differences consecutive poses, so the Pi never needs to
know the module layout.

## Running cameras across two Pis

You can split the cameras across more than one Pi (say, to keep PhotonVision from
maxing out a single board). pi-service doesn't care which Pi a camera is
physically on. Every PhotonVision instance publishes to the RIO's NetworkTables,
and pi-service subscribes by camera name, so a camera on a second Pi shows up the
same as a local one.

Practically: list every camera you want fused in `CAMERAS` regardless of which Pi
it's plugged into, point every Pi's PhotonVision at the RIO, and keep the camera
names unique. Nothing in this program changes.

Worth knowing: the fusion itself is light. It's a small graph solved
single-threaded at 50 Hz, so it isn't what stresses a Pi. PhotonVision is the
heavy part, so balance the cameras for PhotonVision's sake, not the fusion's.

## Troubleshooting

**CMake can't find GTSAM (red includes in CLion, or `find_package(GTSAM)`
fails).** GTSAM isn't installed. Build and install it (step 1), then reload the
CMake project.

**GTSAM build fails with a CMake policy error.** Add
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to the GTSAM `cmake` line.

**Linker can't find `tbb`.** Your GTSAM got built with TBB on. Either install TBB
so it links, or (better) rebuild GTSAM with a clean build directory and
`-DGTSAM_USE_TBB=OFF`.

**`./gradlew :pi-service:jar` fails on a `linuxarm64` artifact.** You're offline
or behind a school firewall. The arm64 native libraries have to download once.
Build it on a normal network connection. `compileJava` still works offline.

**No fused pose on the RIO.** Check the Pi service is running
(`systemctl status poselink`), that both the Pi and RIO are on the robot network
with the right IPs, and that PhotonVision is actually seeing tags. Remember the
service withholds output until it's anchored.
