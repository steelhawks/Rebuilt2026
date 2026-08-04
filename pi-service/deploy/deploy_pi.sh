#!/usr/bin/env bash
#
# Deploy the PoseLink service to the Orange Pi over SSH: build the fat jar
# locally, sync it + the native source, build libposelink_gtsam.so on the Pi, and
# restart the systemd service.
#
# First-time setup on the Pi (once):
#   - install JDK 17 (aarch64), cmake, build-essential
#   - build + install GTSAM (see pi-service/README.md, GTSAM_USE_TBB=OFF)
#   - sudo cp deploy/poselink.service /etc/systemd/system/
#     sudo systemctl daemon-reload && sudo systemctl enable poselink
#
# Override any of these via env, e.g.  PI_HOST=10.26.1.11 ./deploy_pi.sh
set -euo pipefail

PI_USER="${PI_USER:-photon}"
PI_HOST="${PI_HOST:-10.26.1.11}"
DEPLOY_DIR="${DEPLOY_DIR:-/home/photon/poselink}"
SERVICE="${SERVICE:-poselink}"
PHOTON_JAR="${PHOTON_JAR:-/opt/photonvision/photonvision.jar}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${PI_USER}@${PI_HOST}"

# POSELINK_JAR is set when Gradle already built the jar (the deployPi task); build
# it ourselves otherwise so the script still works standalone.
NATIVES="${REPO_ROOT}/pi-service/build/piNatives"

if [ -n "${POSELINK_JAR:-}" ]; then
    JAR="${POSELINK_JAR}"
    echo "==> Using prebuilt jar: ${JAR}"
else
    echo "==> Building Pi service jar (needs internet for arm64 natives)"
    "${REPO_ROOT}/gradlew" :pi-service:jar :pi-service:piNatives
    JAR="$(ls -t "${REPO_ROOT}"/pi-service/build/libs/pi-service*.jar | head -1)"
    echo "    jar: ${JAR}"
fi

if [ ! -d "${NATIVES}" ]; then
    echo "!! ${NATIVES} missing - run ./gradlew :pi-service:piNatives" >&2
    exit 1
fi

echo "==> Preparing deploy dir on ${TARGET}"
ssh "${TARGET}" "mkdir -p '${DEPLOY_DIR}' '${DEPLOY_DIR}/cpp' '${DEPLOY_DIR}/lib'"

echo "==> Syncing jar + arm64 natives + native source"
rsync -az "${JAR}" "${TARGET}:${DEPLOY_DIR}/poselink.jar"
# lib/ holds the WPILib arm64 .so files that java.library.path points at.
rsync -az --delete "${NATIVES}/" "${TARGET}:${DEPLOY_DIR}/lib/"
rsync -az --delete \
    --exclude 'build/' --exclude 'cmake-build-*/' \
    "${REPO_ROOT}/pi-service/cpp/" "${TARGET}:${DEPLOY_DIR}/cpp/"

# photonlib's PhotonCamera constructor unconditionally starts a TimeSyncServer,
# which needs libphotontargetingJNI.so - and PhotonVision publishes
# photontargeting-cpp for athena / x86-64 / osx only, never linuxarm64. So take it
# out of the PhotonVision install that's already on this Pi: same version as the
# server we're talking to, by construction. Must run after the lib/ rsync above,
# which is --delete.
echo "==> Extracting photontargeting natives from ${PHOTON_JAR}"
ssh "${TARGET}" "python3 - '${PHOTON_JAR}' '${DEPLOY_DIR}/lib'" <<'PY'
import os
import sys
import zipfile

jar, dest = sys.argv[1], sys.argv[2]
if not os.path.exists(jar):
    sys.exit("!! %s not found - is PhotonVision installed on this Pi?" % jar)

want = {"libphotontargetingJNI.so", "libphotontargeting.so"}
found = set()
with zipfile.ZipFile(jar) as z:
    for entry in z.namelist():
        base = entry.rsplit("/", 1)[-1]
        if entry.startswith("linux/arm64/") and base in want:
            out = os.path.join(dest, base)
            with z.open(entry) as src, open(out, "wb") as dst:
                dst.write(src.read())
            os.chmod(out, 0o755)
            found.add(base)

missing = want - found
if missing:
    sys.exit("!! not in %s: %s" % (jar, ", ".join(sorted(missing))))
print("    extracted: " + ", ".join(sorted(found)))
PY

echo "==> Building native shim on the Pi"
ssh "${TARGET}" "cmake -S '${DEPLOY_DIR}/cpp' -B '${DEPLOY_DIR}/cpp/build' \
        -DCMAKE_BUILD_TYPE=Release -DPOSELINK_BUILD_TESTS=OFF \
    && cmake --build '${DEPLOY_DIR}/cpp/build' \
    && cp '${DEPLOY_DIR}/cpp/build/libposelink_gtsam.so' '${DEPLOY_DIR}/'"

echo "==> Restarting ${SERVICE}"
ssh "${TARGET}" "sudo systemctl restart '${SERVICE}' \
    && systemctl --no-pager --lines=8 status '${SERVICE}' || true"

echo "==> Done. Follow logs: ssh ${TARGET} journalctl -u ${SERVICE} -f"
