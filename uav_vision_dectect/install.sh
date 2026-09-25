#!/usr/bin/env bash
# Installs native C++ dependencies for the uav_vision_dectect ROS 2 package.
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DEFAULT_ORT_VERSION="1.16.3"
ORT_VERSION="${ONNXRUNTIME_VERSION:-$DEFAULT_ORT_VERSION}"
ORT_PREFIX="${ONNXRUNTIME_ROOT:-}"
INSTALL_SYSTEM_PACKAGES=true

usage() {
    cat <<'EOF'
Usage: ./install.sh [options]

Install the native C++ dependencies needed by uav_vision_dectect.

Options:
  --onnxruntime-root PATH  Install ONNX Runtime below PATH.
  --onnxruntime-version V  Install a CPU ONNX Runtime release version.
  --skip-system-packages   Do not install APT packages.
  -h, --help               Show this help.

The default installs the CPU ONNX Runtime archive in:
  ~/.local/onnxruntime/1.16.3

GPU ONNX Runtime is intentionally not installed by this script. Its CUDA/cuDNN
version must match the NVIDIA driver and CUDA runtime on the host.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --onnxruntime-root)
            ORT_PREFIX="$2"
            shift 2
            ;;
        --onnxruntime-version)
            ORT_VERSION="$2"
            shift 2
            ;;
        --skip-system-packages)
            INSTALL_SYSTEM_PACKAGES=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "$ORT_PREFIX" ]]; then
    ORT_PREFIX="$HOME/.local/onnxruntime/$ORT_VERSION"
fi

if [[ "$INSTALL_SYSTEM_PACKAGES" == true ]]; then
    if ! command -v sudo >/dev/null; then
        printf 'sudo is required to install system packages. Re-run with --skip-system-packages after installing them.\n' >&2
        exit 1
    fi

    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        libopencv-dev \
        ros-humble-cv-bridge \
        ros-humble-sensor-msgs
fi

if [[ -f "$ORT_PREFIX/include/onnxruntime_cxx_api.h" && -f "$ORT_PREFIX/lib/libonnxruntime.so" ]]; then
    printf 'ONNX Runtime is already installed at %s\n' "$ORT_PREFIX"
else
    if ! command -v curl >/dev/null; then
        printf 'curl is required to download ONNX Runtime.\n' >&2
        exit 1
    fi

    archive_name="onnxruntime-linux-x64-${ORT_VERSION}.tgz"
    archive_url="https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${archive_name}"
    temp_dir="$(mktemp -d)"
    trap 'rm -rf "$temp_dir"' EXIT

    printf 'Downloading ONNX Runtime CPU %s...\n' "$ORT_VERSION"
    curl --fail --location --retry 3 --output "$temp_dir/$archive_name" "$archive_url"
    tar -xzf "$temp_dir/$archive_name" -C "$temp_dir"

    mkdir -p "$(dirname "$ORT_PREFIX")"
    rm -rf "$ORT_PREFIX"
    mv "$temp_dir/onnxruntime-linux-x64-${ORT_VERSION}" "$ORT_PREFIX"
fi

printf '\nDependencies are ready. Build with:\n'
printf '  cd %q\n' "$SCRIPT_DIR/.."
printf '  source /opt/ros/humble/setup.bash\n'
printf '  colcon build --packages-select uav_common_msg uav_vision_dectect --cmake-args -DONNXRUNTIME_ROOT=%q\n' "$ORT_PREFIX"
printf '\nFor execution, expose the native shared library:\n'
printf '  export LD_LIBRARY_PATH=%q:${LD_LIBRARY_PATH:-}\n' "$ORT_PREFIX/lib"