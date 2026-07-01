| Model Name | Use | Initial Model Weights | Data Used | Augmentation Applied |
|---|---|---|---|---|
| best_alex.pt and best_alex.onnx | Buoy detection and color classification in isaacSIM | n/a | IsaacSIM images | None |
| best_rf.pt | Real buoy detection and color classification | best_alex.pt | Images taken from live feed of red/green buoys from ZED cameras | All + random |
| one_beacon.pt | Real beacon detection | yolo26n.pt | Images taken from 4 video feeds of drone observing light beacons (videos taken separately for each color) + background images with no beacon | None |
| one_beacon625.pt | Real beacon detection | one_beacon.pt | Images used in one_beacon.pt as well as images captured from live drone footage of beacon on modalAI drone| None |
| one_beacon630.pt | Real beacon detection | one_beacon625.pt | Images used in one_beacon625.pt as well as images captured in another environment from live drone footage of beacon on modalAI drone| None |
| best_crop.pt | Real light area of beacon detection | best_crop_reduced2.pt | All cropped images using labeled area from original dataset | None |
| best_crop630.pt | Real light area of beacon detection | best_crop360.pt | Cropped images of beacon only from images used to train one_beacon630.pt | None |
| SIM_rgb.pt | Beacon color classification in IsaacSIM with domain randomization | n/a | IsaacSIM images with domain randomization | None |
| SIM_beacontop.pt | Beacon + top classification in IsaacSIM | n/a | IsaacSIM images with domain randomization | None |
| SIM_rgb_real-tuned.pt | Sim2real beacon color classification | SIM_rgb.pt | IsaacSIM images with domain randomization + real images | Only on real images |
| SIM_beacon_only.pt | Beacon-only classification in IsaacSIM | n/a | Same as SIM_beacontop.pt | None |
| SIM_top_only.pt | Light area of beacon detection | n/a | Same as SIM_beacontop.pt | None |

Note that all the SIM_ models are currently only trained on non-flashing beacon images.
