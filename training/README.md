# Fine-tuning the beacon model on field frames

Fine-tunes the 2-class pose model `models/YOLOv11n_diamond_detect.pt` (classes
`beacon`, `diamond`, 4 keypoints; trained 2026-11-08) on `tracking_front` frames
from a field run, then exports a `.tflite` for `beacon_overlay`.

Why this model: on the Course Charlie frames it was the only one of the three
checkpoints whose boxes landed on the real beacons (at conf 0.03-0.07). The
deployed 1-class `models/YOLOv11n_diamond_detect_old.pt` boxed trees and sky.
Fine-tuning should mainly raise that confidence.

## 1. Propose boxes (any machine with Python, OpenCV)

```bash
python3 training/prelabel_beacons.py \
    "<...>/Robonation_real_Data/images" -o labels_charlie_v1 \
    --every 3 --start 20261111_014908 --end 20261111_015117 --preview
```

* Writes `images/`, `labels/` (YOLO detection, class 0 = `beacon`) and
  `classes.txt` / `obj.names`; `--preview` draws the boxes for a quick look.
* Expect about 12 candidates per frame on Charlie. It boxes some non-beacons
  (white floating platforms, dock signs, a small boat) and misses dim beacons
  (e.g. grey beacons beside a sun-glint patch). The output folder must be new.

## 2. Review the boxes (the step that matters)

Easiest: the included reviewer (Python 3.10 + OpenCV, nothing to install):

```bash
py -3.10 training/review_boxes.py labels_charlie_v1 -o labels_charlie_v1_reviewed
```

Drag to add a box, right-click to delete, `->`/`<-` for next/previous (saves),
`2`/`3` zoom into the left/right half of the water, `F` whole frame, `X` to
exclude a frame, `Q` to quit. It only reads `labels_charlie_v1`, saves every
frame you leave into `labels_charlie_v1_reviewed`, and resumes where you
stopped. `labels_charlie_v1_reviewed` is then the input to step 3.

Or open `labels_charlie_v1` in another box tool:

* **CVAT**: create a task with the images, label `beacon`, then
  *Upload annotations* -> "YOLO 1.1" (zip `obj.names`, `labels/` as `obj_train_data/`)
  or "Ultralytics YOLO Detection 1.0"; export the same format afterwards.
* **labelImg**: `labelImg labels_charlie_v1/images labels_charlie_v1/classes.txt labels_charlie_v1/labels`
  (YOLO mode).
* **Roboflow**: upload images + YOLO TXT labels, export "YOLOv8 / YOLO TXT".

Rules:

* Box **every** visible beacon -- an unlabelled beacon teaches the model that
  beacons are background. Far beacons only a few pixels big: box them if you
  can tell they are beacons, otherwise leave the frame out.
* Box the whole beacon (base to tip), as `beacon` (class 0).
* Delete boxes on anything that isn't a beacon.
* Frames with no beacons (dock, shore) are useful negatives: keep them with an
  empty label file.

Put the reviewed result back as `<reviewed>/images/*.jpg` + `<reviewed>/labels/*.txt`.

## 3. Build the training set

```bash
python3 training/build_dataset.py labels_charlie_v1_reviewed -o ds_charlie_v1 \
    --val-start 20261111_015050 --val-end 20261111_015117
```

* Stretches frames to 640x640 like `beacon_overlay --resize stretch` (the model
  was trained that way -- do not letterbox).
* Writes pose labels (`beacon` boxes get 4 invisible keypoints) and
  `data.yaml` with `kpt_shape [4, 3]` and `flip_idx [0, 3, 2, 1]`.
* Splits by time: keep a whole time block for val, never random frames.

## 4. Fine-tune (GPU machine if possible; CPU works, slowly)

On Windows (PowerShell) the `yolo` command is `yolo.exe` in Python's Scripts folder
(`python -m ultralytics` does NOT work):

```powershell
& "$env:LOCALAPPDATA\Programs\Python\Python310\Scripts\yolo.exe" pose train model=models\YOLOv11n_diamond_detect.pt data="<full path>\ds_charlie_v1\data.yaml" imgsz=640 epochs=60 patience=20 freeze=10 lr0=0.001 mosaic=1.0 fliplr=0.5 hsv_v=0.4 scale=0.5 batch=8 device=cpu workers=0 project=runs_beacon name=charlie_v1
```

Linux / WSL:

```bash
pip install ultralytics   # the model was trained with 8.4.x
yolo pose train model=models/YOLOv11n_diamond_detect.pt data=ds_charlie_v1/data.yaml \
     imgsz=640 epochs=60 patience=20 freeze=10 lr0=0.001 \
     mosaic=1.0 fliplr=0.5 hsv_v=0.4 scale=0.5 \
     project=runs_beacon name=charlie_v1
```

* `freeze=10` keeps the backbone (the features that find diamonds up close)
  fixed, and `lr0=0.001` keeps changes small: there is no original training set
  to mix in, so this limits forgetting.
* Smoke-tested (1 epoch, CPU, pre-labelled boxes): weights load 541/541,
  `best.pt` is written; `pose_loss` is 0 when only `beacon` boxes are labelled
  (expected -- they have no visible keypoints).

**Check before using it** -- compare `models/YOLOv11n_diamond_detect.pt` and
`runs_beacon/charlie_v1/weights/best.pt` on the held-out val block: beacons
found at conf >= 0.5 should go up a lot, and boxes off the water (trees, sky,
shore) should not. If you have any close-range frames with diamonds, check the
new model still finds their diamonds and corners there.

## 5. Export to TFLite (Linux / WSL; versions must match the drone)

```bash
python3 -m venv ~/.venvs/yolo-export && source ~/.venvs/yolo-export/bin/activate
pip install ultralytics==8.4.55 tensorflow==2.19.0 onnx==1.17.0
cp runs_beacon/charlie_v1/weights/best.pt models/YOLOv11n_beacon_charlie_v1.pt
yolo export model=models/YOLOv11n_beacon_charlie_v1.pt format=tflite imgsz=640 half=True
# -> models/YOLOv11n_beacon_charlie_v1_saved_model/YOLOv11n_beacon_charlie_v1_float16.tflite
```

TensorFlow 2.19 to match the drone's TFLite runtime (a newer exporter can emit
ops it doesn't know); `onnx` 1.17.0 because newer needs a protobuf TF 2.19
rejects. Use a **new file name** so the current models stay as they are.

### If the export dies with `free(): double free detected` (WSL with an NVIDIA GPU)

Seen 2026-11-12 on WSL Ubuntu 22.04 with a laptop NVIDIA GPU. `onnx2tf`
(1.28.8, `onnx2tf.py` line ~954) sets `CUDA_VISIBLE_DEVICES=-1`; with WSL's
Windows CUDA driver (`/usr/lib/wsl/lib/libcuda.so`) that makes TensorFlow crash
while scanning devices. Fix: start TensorFlow *before* importing onnx2tf, and run
only the ONNX -> TFLite step in a clean venv (no PyTorch, no ROS):

```bash
sudo apt install -y python3.10-venv                       # once
python3 -m venv ~/.venvs/tfexport                          # once
~/.venvs/tfexport/bin/pip install tensorflow==2.19.0 tf_keras==2.19.0 onnx==1.17.0 \
    onnx2tf==1.28.8 onnx_graphsurgeon sng4onnx onnxruntime ai-edge-litert psutil   # once

# The .onnx comes from the (crashed) `yolo export ... format=tflite` above -- it is
# written before the crash and is the right one for TFLite (don't use format=onnx).
cd ~/export     # folder holding YOLOv11n_beacon_charlie_v1.onnx (Linux filesystem)
env -i HOME="$HOME" PATH=/usr/bin:/bin ~/.venvs/tfexport/bin/python -c "
import tensorflow as tf; tf.config.list_physical_devices(); tf.constant(0)   # init TF first
import onnx2tf
onnx2tf.convert(input_onnx_file_path='YOLOv11n_beacon_charlie_v1.onnx',
    output_folder_path='YOLOv11n_beacon_charlie_v1_saved_model', not_use_onnxsim=True,
    verbosity='error', output_signaturedefs=True, disable_group_convolution=True,
    enable_batchmatmul_unfold=True)"
```

Check the result: input `[1, 640, 640, 3]` float32, output `[1, 18, 8400]`
(4 box + 2 class scores + 4 x 3 keypoints). For `charlie_v1` the float16
`.tflite` matched the `.pt` on the held-out frames (recall at conf 0.4: 80/203
both; false boxes 19 vs 18).

## 6. Deploy

```bash
scp models/YOLOv11n_beacon_charlie_v1_saved_model/YOLOv11n_beacon_charlie_v1_float16.tflite \
    root@<drone-ip>:/data/models_eval/
# on the drone -- both settings matter:
MODEL=YOLOv11n_beacon_charlie_v1_float16.tflite CLASSES=beacon,diamond \
    voxl/beacon_overlay/run.sh start
```

`CLASSES=beacon,diamond` must match the 2-class model: with a wrong list the
detector labels class 0 with the first name, and beacon_overlay treats
diamonds as beacons (or the reverse). Check `http://<drone-ip>:8091/`: beacons
should show `beacon` boxes with `box` range fixes.
