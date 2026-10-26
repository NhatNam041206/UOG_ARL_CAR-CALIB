# Cross-line (tile) counter

`vision/cross_line_counter.py` counts the transverse tile seams that pass under
the car. It is the first step of positioning: tiles crossed x tile size =
distance along the corridor. It has no motion authority: the count goes to the
route CSV (`cross_tile_count`, `cross_counted`, `cross_lines`, `cross_tracks`),
the HUD (`TILES n`, top right) and `route_summary.json`
(`extra_meta.cross_tile_count`). The count resets when a route starts.

## How it works

1. **Dark-line filter.** Black-hat with a vertical kernel
   (`CROSS_BLACKHAT_KERNEL_PX`) keeps thin dark lines that run roughly
   horizontally (grout) and drops glare spots and broad shadows. Thresholded at
   `CROSS_DARK_THRESH`.
2. **Detection in the look-ahead ROI** (`CROSS_ROI_TOP` .. `CROSS_ROI_BOTTOM`,
   ratios of frame height). Standard Hough restricted to
   +-`CROSS_MAX_TILT_DEG` around horizontal; a line needs
   `CROSS_MIN_LEN` x frame width votes, so faint seams that span the frame
   still pass. Each line is `(y at the centre column, tilt)`.
3. **Tracking.** A line is matched to a track by predicted y
   (`CROSS_TRACK_MAX_DY`) and tilt (`CROSS_SLOPE_TOL_DEG`). Seams only move down
   the image while driving forward. A track is confirmed after
   `CROSS_CONFIRM_FRAMES` detections ("detected lines list"). Two tracks on
   the same seam collapse into the older one.
4. **Counter band** (`CROSS_COUNTER_Y` +- `CROSS_COUNTER_H`/2). A confirmed
   track whose y crosses `CROSS_COUNTER_Y` counts one tile. Near the bottom of
   the frame seams are smeared by motion blur, so a track that is lost keeps
   moving at its last speed for up to `CROSS_MAX_MISS_FRAMES` and can still
   count. A count needs at least `CROSS_COUNT_MIN_GAP_FRAMES` frames and
   `CROSS_COUNT_MIN_GAP_RATIO` x the median recent interval since the previous
   one (rejects ghost lines and shadow edges next to a real seam).

The counter line sits at 0.45 rather than near the bottom because the camera-2
recordings lose most seams below ~0.5 of the frame height to motion blur. Any
row works as long as it stays fixed: every seam crosses it exactly once.

## Parameters

All are env vars (`CROSS_*`) and live in the dashboard Tune tab, group
"Cross lines" (except `CROSS_MERGE_DY` / `CROSS_MERGE_DTILT_DEG`, env only).

| Env | Default | Meaning |
| --- | --- | --- |
| `CROSS_ENABLED` | true | Run the counter |
| `CROSS_ROI_TOP` / `CROSS_ROI_BOTTOM` | 0.12 / 0.38 | Look-ahead ROI where new seams are picked up |
| `CROSS_COUNTER_Y` / `CROSS_COUNTER_H` | 0.45 / 0.10 | Counter line and band height |
| `CROSS_BLACKHAT_KERNEL_PX` | 15 | Dark-line kernel height (wider than a blurred seam) |
| `CROSS_DARK_THRESH` | 6 | Dark-line threshold (grey levels) |
| `CROSS_MIN_LEN` | 0.35 | Hough votes needed, x frame width |
| `CROSS_MAX_TILT_DEG` | 35 | Max tilt of a transverse seam (car yaw) |
| `CROSS_SLOPE_TOL_DEG` | 8 | Tilt change allowed between frames for one track |
| `CROSS_MERGE_DY` / `CROSS_MERGE_DTILT_DEG` | 0.05 / 6 | Lines this close are one seam |
| `CROSS_TRACK_MAX_DY` | 0.12 | Max y jump per frame for one track |
| `CROSS_CONFIRM_FRAMES` | 2 | Detections before a track is confirmed |
| `CROSS_MAX_MISS_FRAMES` | 8 | Frames a lost track keeps coasting |
| `CROSS_COUNT_MIN_GAP_FRAMES` | 4 | Min frames between two counts |
| `CROSS_COUNT_MIN_GAP_RATIO` | 0.5 | Min gap as a fraction of the median recent interval (0 = off) |

## Offline replay

```bash
python scripts/replay_cross_lines.py <route_dir> [<route_dir> ...] --video --out cross_replay
python scripts/replay_cross_lines.py <route_dir> --set counter_y=0.5 --set dark_thresh=8
```

Writes `cross_lines.csv`, `cross_summary.json` and, with `--video`,
`cross_lines.mp4` (left: overlay, right: dark-line mask) per route.

## Known limits

- One missed or doubled seam is one tile (tile size) of error, and the count
  does not correct itself. Room-sign anchors are the planned correction.
- Furniture bars and shadow edges parallel to the seams can count
  (seen when the car drives under a bench).
- Reversing is not handled: seams moving up the image are not tracked.
- The tile size is not applied here; distance = count x measured tile size.

## Placing the count on the campus map

`positioning/tile_position.py` maps the count onto the campus tile map
(`routes/tile_maps/campus-path.json`, a JSON copy of
`NhatNam041206/ARL_AUTOCAR` `routes/tile_maps/campus-path.yaml`; a `.yaml` map
also loads when PyYAML is installed). Cells are `(row, column)`; row grows
south (down on the map image), column grows east.

- **Straight:** `--start <anchor|row,col> --heading east` → cell = start + N
  steps east.
- **Path:** `--start AAD-1 --goal gym-01` → shortest 4-connected map path,
  cell = path[N]. The count follows the car, so the index keeps advancing
  through turns (a corner can be one tile off).
- A predicted cell outside the runnable cells is `CONTRADICTORY` (never
  snapped); counting beyond the goal is `PAST_END`.
- `--anchor FRAME:REF` (repeatable) says the car is at that anchor or cell at
  that frame; counting continues from there.

```bash
python scripts/replay_tile_position.py <route_dir> --start AAD-1 --goal gym-01 --out tile_position_replay
```

Writes `tile_position.csv` (per frame: count, path index, row, column,
status), `tile_position_map.png` and `tile_position_summary.json`. Offline
only; nothing on the robot uses the map yet.

First validation run: AAD-1 → gym-01 is 16 tiles straight east on the map.
