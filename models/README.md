| Model Name | Use | Initial Model Weights | Data Used | Augmentation Applied |
|---|---|---|---|---|
| square_beacon.pt | Updated model for square top beacons | beacon_full86.pt | Video footage of beacon in a pool recorded with DJI drone | foggy, dim, overcast |
| beacon_square_top.pt | Light area of square beacon detection | beacon_top87iv.pt | Cropped beacon images from beacon top video used for square_beacon.pt | None |
| best_square3_914.pt | Square beacon with four classes (red, green, blue, unkown) | best_beacon_square3.pt | Images of beacon taken on modalAI starling2 max | None |
| best_square3.tflite | tflite model of Square beacon with four classes (red, green, blue, unkown) | best_square3_914.pt | mages of beacon taken on modalAI starling2 max | None |

