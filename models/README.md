| Model Name | Use | Initial Model Weights | Data Used | Augmentation Applied |
|---|---|---|---|---|
| best_alex.pt and best_alex.onnx | Buoy detection and color classification in isaacSIM | n/a | IsaacSIM images | None |
| best_rf.pt | Real buoy detection and color classification | best_alex.pt | Images taken from live feed of red/green buoys from ZED cameras | All + random |
| one_beacon.pt | Real beacon detection | yolo26n.pt | Images taken from 4 video feeds of drone observing light beacons (videos taken separately for each color) + background images with no beacon | None |
| best_crop.pt | Real light area of beacon detection | best_crop_reduced2.pt | All images and labels from original dataset | None |
| SIM_rgb.pt | Beacon color classification in IsaacSIM with domain randomization | n/a | IsaacSIM images with domain randomization | None |
| SIM_beacontop.pt | Beacon + top classification in IsaacSIM | n/a | IsaacSIM images with domain randomization | None |
