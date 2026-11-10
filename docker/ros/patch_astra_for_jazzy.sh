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
grep -rhoE '#include[[:space:]]*<[a-z0-9_]+/[A-Za-z0-9_/]+\.h>' "$src_dir" \
    | sed -E 's/.*<(.*)>/\1/' | sort -u | while read -r hdr; do
    if ! find_header "$hdr" && find_header "${hdr}pp"; then
        echo "patch: <$hdr> -> <${hdr}pp>"
        grep -rlF "<$hdr>" "$src_dir" | xargs sed -i "s#<$hdr>#<${hdr}pp>#g"
    fi
done

# rclcpp (Jazzy) dropped the old alias OnParametersSetCallbackType; the
# same callback type is now called OnSetParametersCallbackType.
grep -rlF "OnParametersSetCallbackType" "$src_dir" | xargs -r sed -i \
    's/NodeParametersInterface::OnParametersSetCallbackType/NodeParametersInterface::OnSetParametersCallbackType/g'
echo "patch: OnParametersSetCallbackType -> OnSetParametersCallbackType"
