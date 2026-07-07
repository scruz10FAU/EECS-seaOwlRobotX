# Beacon Detector Config Files

All configs are used with `beacon_detector_config.py`. Pass a config with `--config <file>`.

| Config file | Platform / use case | Camera | Image topic | Target color | Target blinking | True dist (m) |
|---|---|---|---|---|---|---|
| `beacon_config.json` | Template / default | ZED2 | `/zed/zed_node/rgb/...` | — | — | 0.4826 |
| `beacon_config_physical.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | — | — | — |
| `beacon_config_sim.json` | Isaac Sim (live ROS) | Pegasus front cam | `/iris_0/front_cam/rgb` | — | — | — |
| `modal_config.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | — | true | 0.4826 |
| `modal_config_fblue.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | blue | false | 0.4826 |
| `modal_config_fgreen.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | green | false | 0.4826 |
| `modal_config_fred.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | red | false | 0.4826 |
| `modal_config_tblue.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | blue | true | 0.4826 |
| `modal_config_tgreen.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | green | true | 0.4826 |
| `modal_config_tred.json` | ModalAI Starling 2 (live ROS) | VOXL2 hires front | `/hires_front_small_color` | red | true | 0.4826 |
| `bb_beacon_config.json` | Video file (`blink_blue.mp4`) | ZED2 | `/zed/zed_node/rgb/...` | — | — | 0.4826 |
| `gb_beacon_config.json` | Video file (`blink_green.mp4`) | ZED2 | `/zed/zed_node/rgb/...` | — | — | 0.4826 |
| `rb_beacon_config.json` | Video file (`blink_red.mp4`) | ZED2 | `/zed/zed_node/rgb/...` | — | — | 0.4826 |
| `rv_beacon_config.json` | Video file (`reduced_DJI.mp4`) | ZED2 | `/zed/zed_node/rgb/...` | — | — | 0.4826 |

**Target color / blinking** — when set, the `target_color`, `target_blinking`, and `target_match` columns are populated in the CSV log so detections can be checked against the known ground truth. `—` means the field is `null` and those columns will be empty.

**True dist** — `true_dist` is the known ground-truth distance to the beacon in metres, used for validation. `—` means the field is unset in that config.

## Naming convention

| Prefix | Meaning |
|---|---|
| `modal_` | ModalAI VOXL2 live ROS mode |
| `bb_` / `gb_` / `rb_` | Video file — blinking blue / green / red beacon |
| `rv_` | Video file — DJI drone footage |
| `_f<color>` suffix | Steady (fixed) beacon of that color |
| `_t<color>` suffix | Blinking beacon of that color |
