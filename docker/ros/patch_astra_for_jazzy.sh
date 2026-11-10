#!/bin/bash
# Make ros2_astra_camera (written for Humble) build on Jazzy.
# 1) It includes ROS headers as "<pkg/x.h>".
# Jazzy removed several of those .h shims in favour of ".hpp". Rewrite an
# include to ".hpp" only when the .h is missing and the .hpp exists.
set -euo pipefail
src_dir="$1"
inc_root="/opt/ros/${ROS_DISTRO}/include"
find_header() {  # $1 = "pkg/name.ext" -> 0 if installed
    local pkg="${1%%/*}"
    [[ -f "$inc_root/$pkg/$1" || -f "$inc_root/$1" ]]
}
{ grep -rhoE '#include[[:space:]]*<[a-z0-9_]+/[A-Za-z0-9_/]+\.h>' "$src_dir" || true; } \
    | sed -E 's/.*<(.*)>/\1/' | sort -u | while read -r hdr; do
    if ! find_header "$hdr" && find_header "${hdr}pp"; then
        echo "patch: <$hdr> -> <${hdr}pp>"
        grep -rlF "<$hdr>" "$src_dir" | xargs sed -i "s#<$hdr>#<${hdr}pp>#g"
    fi
done

# rclcpp (Jazzy) dropped the old alias OnParametersSetCallbackType; the
# same callback type is now called OnSetParametersCallbackType.
{ grep -rlF "OnParametersSetCallbackType" "$src_dir" || true; } | xargs -r sed -i \
    's/NodeParametersInterface::OnParametersSetCallbackType/NodeParametersInterface::OnSetParametersCallbackType/g'
echo "patch: OnParametersSetCallbackType -> OnSetParametersCallbackType"

# GCC 13 (Ubuntu 24.04) no longer pulls <cstdint> in transitively. Add it to
# every C++ file of the driver itself that uses fixed-width int types without
# including it. Skip the vendored openni2 headers: they are fine as shipped and
# start with a UTF-8 BOM, so a line put in front of it breaks them.
{ grep -rlE '\bu?int(8|16|32|64)_t\b' "$src_dir" --exclude-dir=openni2 --include='*.h' --include='*.hpp' --include='*.cpp' || true; } \
    | { xargs -r grep -LE '#include[[:space:]]*<(cstdint|stdint\.h)>' || true; } \
    | while read -r f; do
        sed -i -e '1s/^\xEF\xBB\xBF//' -e '1i #include <cstdint>' "$f"
        echo "patch: +<cstdint> $f"
    done
