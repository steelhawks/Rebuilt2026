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

PI_USER="${PI_USER:-orangepi}"
PI_HOST="${PI_HOST:-10.26.1.11}"
DEPLOY_DIR="${DEPLOY_DIR:-/home/orangepi/poselink}"
SERVICE="${SERVICE:-poselink}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${PI_USER}@${PI_HOST}"

echo "==> Building Pi service jar (needs internet for arm64 natives)"
"${REPO_ROOT}/gradlew" :pi-service:jar
JAR="$(ls -t "${REPO_ROOT}"/pi-service/build/libs/pi-service*.jar | head -1)"
echo "    jar: ${JAR}"

echo "==> Preparing deploy dir on ${TARGET}"
ssh "${TARGET}" "mkdir -p '${DEPLOY_DIR}' '${DEPLOY_DIR}/cpp'"

echo "==> Syncing jar + native source"
rsync -az "${JAR}" "${TARGET}:${DEPLOY_DIR}/poselink.jar"
rsync -az --delete \
    --exclude 'build/' --exclude 'cmake-build-*/' \
    "${REPO_ROOT}/pi-service/cpp/" "${TARGET}:${DEPLOY_DIR}/cpp/"

echo "==> Building native shim on the Pi"
ssh "${TARGET}" "cmake -S '${DEPLOY_DIR}/cpp' -B '${DEPLOY_DIR}/cpp/build' \
        -DCMAKE_BUILD_TYPE=Release -DPOSELINK_BUILD_TESTS=OFF \
    && cmake --build '${DEPLOY_DIR}/cpp/build' \
    && cp '${DEPLOY_DIR}/cpp/build/libposelink_gtsam.so' '${DEPLOY_DIR}/'"

echo "==> Restarting ${SERVICE}"
ssh "${TARGET}" "sudo systemctl restart '${SERVICE}' \
    && systemctl --no-pager --lines=8 status '${SERVICE}' || true"

echo "==> Done. Follow logs: ssh ${TARGET} journalctl -u ${SERVICE} -f"
