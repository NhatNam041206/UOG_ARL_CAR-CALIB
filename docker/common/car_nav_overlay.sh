#!/bin/bash
# Source-overlay for the car_nav ROS packages (sourced by the entrypoints).
#
# The image ships a baked copy of the packages in /opt/car_nav_ws. When the
# repo's package folders are bind-mounted under $CAR_NAV_SRC_ROOT (default
# /ws/src) this builds them into an overlay workspace and puts it in front of
# the baked one, so editing code / adding a console script, launch file or
# data file needs only a container restart, never an image rebuild.
#
#   * Pure Python (ament_python): `colcon build` takes seconds, no internet.
#   * Rebuilt only when the mounted files changed (content hash of the tree).
#   * A build goes to a temp dir and replaces the active one only on success.
#     A failed build keeps the last good overlay and logs the error
#     ($OVERLAY_DIR/last_build_error.log). No mounted source -> baked code.
#   * CAR_NAV_OVERLAY=0 turns it off (baked code only).
#
# Env: CAR_NAV_OVERLAY_PKGS (default "car_nav"), CAR_NAV_SRC_ROOT,
#      CAR_NAV_OVERLAY_DIR (default /data/nav/.overlay; a persistent volume).
# Never fails the container start: every problem falls back to the baked code.

car_nav_overlay() {
  local src="${CAR_NAV_SRC_ROOT:-/ws/src}"
  local dir="${CAR_NAV_OVERLAY_DIR:-/data/nav/.overlay}"
  local pkgs="${CAR_NAV_OVERLAY_PKGS:-car_nav}"
  local log="$dir/last_build_error.log"
  local pkg have="" hash tmp new
  say() { echo "[overlay] $*" >&2; }

  [[ "${CAR_NAV_OVERLAY:-1}" == "0" ]] && { say "disabled: using the code baked into the image"; return 0; }
  for pkg in $pkgs; do
    if [[ -f "$src/$pkg/package.xml" && -f "$src/$pkg/setup.py" ]]; then have+="$pkg "; else say "no mounted source for $pkg ($src/$pkg)"; fi
  done
  if [[ -z "$have" ]]; then say "no mounted source: using the code baked into the image"; return 0; fi
  if [[ "$have" != "$pkgs " ]]; then say "a package is missing: using the baked code for all"; return 0; fi
  mkdir -p "$dir/builds" 2>/dev/null || { say "cannot write $dir: using the baked code"; return 0; }

  hash=$( { echo "${ROS_DISTRO:-}"; cd "$src" && for pkg in $have; do
             find "$pkg" -type f ! -path '*/__pycache__/*' ! -name '*.pyc' ! -path "$pkg/build/*" ! -path "$pkg/install/*" ! -path '*.egg-info/*' -print0
           done | sort -z | xargs -0 sha256sum; } 2>/dev/null | sha256sum | cut -c1-16 )
  new="$dir/builds/$hash"

  if [[ ! -f "$new/.ok" ]]; then
    say "source changed ($hash): colcon build (a few seconds)"
    # colcon writes absolute install paths, so build directly under the final name; the
    # ".ok" marker is written only on success, and the active overlay ("current") is
    # untouched until then. A dir without the marker is a failed/interrupted build.
    rm -rf "$new"; mkdir -p "$new/src"
    for pkg in $have; do
      # build from a copy: the mount may be read-only and must not change mid-build
      mkdir -p "$new/src/$pkg"
      ( cd "$src/$pkg" && tar -c --exclude='__pycache__' --exclude='*.pyc' --exclude='build' --exclude='install' --exclude='log' . ) | tar -x -C "$new/src/$pkg"
    done
    if ( set -e; source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"; source /opt/car_nav_ws/setup.bash
         cd "$new"
         timeout "${CAR_NAV_OVERLAY_TIMEOUT:-300}" colcon --log-base "$new/log" build --merge-install            --base-paths "$new/src" --build-base "$new/build" --install-base "$new/install"            --packages-select $have ) >"$dir/last_build.log" 2>&1        && [[ -f "$new/install/setup.bash" ]]; then
      rm -rf "$new/build" "$new/log"
      touch "$new/.ok"
      ln -sfn "builds/$hash" "$dir/current.tmp" && mv -T "$dir/current.tmp" "$dir/current"
      rm -f "$log"
      say "build ok"
    else
      { echo "[$(date -u +%FT%TZ)] colcon build failed for source hash $hash"; tail -n 80 "$dir/last_build.log"; } > "$log" 2>/dev/null
      say "BUILD FAILED, keeping the last good overlay (if any). Error saved in $log:"
      tail -n 15 "$dir/last_build.log" >&2
      rm -rf "$new"
    fi
  else
    ln -sfn "builds/$hash" "$dir/current.tmp" && mv -T "$dir/current.tmp" "$dir/current"
    say "up to date ($hash)"
  fi

  # keep the active build and the newest other one (instant rollback), drop the rest
  local cur; cur=$(readlink "$dir/current" 2>/dev/null | sed 's#.*/##')
  ls -1t "$dir/builds" 2>/dev/null | grep -vx "${cur:-none}" | tail -n +2 | while read -r old; do rm -rf "$dir/builds/$old"; done

  if [[ -f "$dir/current/install/setup.bash" && -f "$dir/current/.ok" ]]; then
    source "$dir/current/install/setup.bash"
    say "active overlay: $(readlink "$dir/current")"
  else
    say "no good overlay: using the code baked into the image"
  fi
}
