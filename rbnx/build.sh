#!/usr/bin/env bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*

echo "Installation completed."

set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
fi

mkdir -p "$PKG/rbnx-build/ws/install"
PROTO_GEN_DEFAULT="$(cd "$PKG/../.." && pwd)/robonix/rust/examples/proto_gen"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export ROBONIX_PROTO_GEN="\${ROBONIX_PROTO_GEN:-$PROTO_GEN_DEFAULT}"
export PYTHONPATH="$PKG:\$ROBONIX_PROTO_GEN:\${PYTHONPATH:-}"
EOF

echo "[nav2-rbnx] build prepared"
