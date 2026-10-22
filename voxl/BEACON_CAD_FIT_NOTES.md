# Beacon CAD fit from diamond keypoints, and range from the box alone

Source: `pose_data_gen/beacon_labeler` (`core/beacon_model.py`, `core/refine.py`). Written 2026-10-19.
Camera: ModalAI Starling 2 Max front tracking, fisheye, 1280x800, fx 445.807, fy 446.150, cx 627.992,
cy 367.084, D = [0.06597, 0.02538, -0.014176, 0.0018888] (OpenCV `cv2.fisheye`). Labels stay in raw
(distorted) pixels; the fit undistorts them internally.

This documents how VLM-detected diamond keypoints are fit to the beacon CAD to get a coarse pose
(used to repair/validate labels), and how range is approximated when only the beacon bbox is valid.
The pose is "coarse" on purpose: it has to be good to ~1 px for labelling, not good enough to fly on.

## 1. Model

Beacon frame: origin at base centre, Z up, metres. Numbers are from NOTES.md / `beacon_pose.py`.

| Quantity | Value |
|---|---|
| Height, base to top | 0.52605 |
| Square skirt at base | 0.559 |
| White section (carries diamonds) | z 0.0782 .. 0.3697 |
| Face width at bottom / top of white section | 0.432 / 0.254 |

Face-0 diamond corners (on the -y side), order `bottom, left, top, right` (= YOLO kpt order):

```
top    ( 0.0     -0.14074  0.32499)
bottom ( 0.0     -0.2      0.13063)
left   (-0.1016  -0.17037  0.22781)
right  ( 0.1016  -0.17037  0.22781)
```

Faces 1-3 are face 0 rotated k*90 deg about Z. Each face also has a panel outline (the trapezoid
the diamond sits on) used for occlusion/visibility. Face normals point outward; a face is visible
if its centre is in front of the camera (z > 0.05) and the normal faces the camera. Beyond that,
faces 75-90 deg off normal are technically visible but edge-on and rarely carry a usable diamond.

Gravity: `captures.csv` quaternion (body FRD -> NED) gives world-up in the camera frame, assuming the
camera looks forward and level (cam z = body x, cam x = body y, cam y = body z). Checked: fitted
beacon-up vs gravity median ~5 deg (a 180 deg flip gave 172 deg, so the sign is right).
The beacon floats, so gravity is only a soft prior on its up axis.

## 2. Coarse fit: diamond keypoints -> beacon pose (R, t)

Input per diamond: 4 corners (VLM), the diamond's stage-2 box, and per-corner confidence.

1. **Trusted corners.** Keep a corner if confidence >= `min_corner_conf` and it lies inside the
   stage-2 box. The VLM often returns the *neighbour's* corners for a thin sliver diamond; those
   fall outside the box and are dropped. With exactly 3 trusted corners, the 4th is completed as a
   parallelogram (median error 0.07 px against full 4-corner diamonds).
2. **Seed pose per diamond (IPPE).** Undistort the corners to normalised coordinates, treat the diamond
   as a planar target and call `cv2.solvePnPGeneric(FACES[0], pts, I, None, SOLVEPNP_IPPE)`. It
   returns two candidate poses (the classic planar flip ambiguity). Try all 4 cyclic rotations of the
   corner names too, because the VLM's naming of a skewed diamond is not reliable.
   Reject seeds where the face is not visible or the beacon-up axis is more than `max_tilt_deg` (45)
   from gravity.
3. **Assign the other diamonds to faces.** Seed diamond is face 0. For each other diamond with >= 3
   trusted corners, search over (free face, name rotation): add its corners, refit the pose
   (`solvePnPRefineLM`, 3 rounds, dropping corners worse than `inlier_px` = 2 px), and keep the
   combination whose rms <= `match_px` (4 px) with both faces visible and tilt OK. A lone face-on
   diamond leaves yaw poorly determined, so a neighbour cannot be predicted from it alone; the joint
   search avoids having to.
   A diamond with no usable corners is matched by box only: project each free face, take the
   predicted bounding box, require front winding, size ratio 0.4-2.5 and IoU >= 0.2.
4. **Pick the best seed.** Score = number of diamonds matched, then -rms, minus 5 per outlier, minus a
   weak gravity term of 0.05 px per degree of tilt. Gravity thus breaks the IPPE ambiguity for a lone
   diamond but never outvotes real corners.
5. **Final fit with slack** (`scipy.least_squares`, `soft_l1`, f_scale 2 px). Unknowns: beacon pose
   (6) + 7 slack parameters per non-reference panel. The reference panel is exact and defines the
   pose. Per-panel slack: rotation about 3 axes (sigma 3 deg), translation (sigma 5 mm per axis),
   scale (sigma 3%). Each is a Gaussian prior costing `prior_px` (1 px) per sigma, so a panel the
   data cannot constrain stays at nominal CAD. Residuals: corner reprojection (px), box-edge
   residuals for box-only panels, and gravity on the up axis with `tilt_sigma_deg` 10.
6. **Gates.** Fail if rms > 3 px, tilt > 45 deg, or inlier fraction < 0.6.

Findings that justify the structure:
- The real beacon is close to CAD. Joint two-diamond fit matches VLM corners to median 0.55 px; slack
  settles at median 0.7 deg / 0.5-1 mm.
- Final fit rms on the 577-image set: median 0.38 px, p90 0.66 px. Tilt vs gravity: median 3.8 deg,
  p90 6.2 deg.
- **Single-diamond limit.** Predicting a hidden second diamond from one diamond + gravity is off by
  median 1.5 px (p90 2.55, ~2.1 px for panels at 60-75 deg). Sliver diamonds are 2-4 px wide, so the
  prediction cannot place them reliably. This is a limit of the technique, not a CAD error, and
  is why projected (never-detected) diamonds are turned off (`project_missing: false`).
- With one diamond the tilt comes out prior-dominated (median 1.0 deg), and the projected beacon box
  is ~3.3 px higher than the VLM box (1-2 px with two diamonds). Treat single-diamond poses as lower
  confidence: filter on `refine.n_diamonds == 1`.

Outputs used downstream: refined diamond corners (CAD-named, so naming follows the model rather
than image geometry; this disagreed with a geometric naming rule on ~22% of two-diamond beacons),
and the beacon bbox = projection of the skirt corners, white-section top corners and the axis top
point (`top_radius_m` 0, margin 1 px).

## 3. Range when only the beacon bbox is valid

No diamonds => no PnP, so range comes from the apparent size of the box and bearing from its position.
The CAD gives the physical size: the box spans skirt to top, H = 0.52605 m. (A box that stopped at
the white section, 0.370 m, would make every range ~42% long.) Width is unusable: it depends on yaw,
0.559 m face-on to ~0.79 m corner-on.

**Pinhole approximation** (matches `estimate_distance_from_bbox` in NOTES.md):

```
slant ~= H * fy / bbox_h          # fy = 446.15 px, bbox_h in px
```

E.g. a 64 px box (the dataset median) is ~3.7 m; 16 px is ~14.7 m; 6 px is ~39 m. Range error is
inversely proportional to height error: +/-1 px on a 16 px box is +/-6% range, on 6 px it is +/-17%.

**Fisheye-correct version** (preferred; this is what `locate_box()` in NOTES.md does):

1. Undistort the box top-centre and bottom-centre pixels with `cv2.fisheye.undistortPoints` to rays
   r_top, r_bot (unit vectors in the camera frame).
2. Apparent angular height `a = angle(r_top, r_bot)`.
3. `slant ~= H_eff / (2 * tan(a / 2))` (use `H_eff / a` for small a).

The pinhole formula is only right on the optical axis; toward the edge of the fisheye image it
mis-scales because pixel size per radian changes with radius.

**Look-down correction.** Seen from elevation `e` (camera above the beacon, looking down), the box also
contains some of the top surface: apparent height = H cos(e) + depth sin(e), with depth = the top
width 0.254 m (CAD white-section top). Solve for the slant using this `H_eff`. Without it range is
short by 4% (2 m altitude, 40 m range) up to 18% (8 m altitude, 30 m range). Elevation `e` comes from
the attitude/altitude, or from the bbox ray itself.

**Ray prior to refine it.** The skirt sits on the water, so the ray through the box bottom-centre
intersects the water plane without a height assumption; push the point back by half the skirt depth
(0.28 m) to get the base centre. The two estimates (size, ray) should agree; the voxl implementation
gates on chi2(2) < 13.82 and rejects the box otherwise (catches false detections and a wrong water
height).

Expected box-only accuracy (synthetic, box edges +/-1 px, attitude 0.5 deg, altitude 0.3 m): about
0.25 m median at 8 m range, 0.66 m at 20 m, 0.86 m at 30 m, 2 m at 45 m. Use a relative sigma of
~0.08 with the look-down correction on, ~0.15 without.

**Caveat for labeler output.** In `pose_data_gen` the beacon bbox is the *model projection* when a
fit exists (so it follows the CAD hull), and the VLM box otherwise. Both obey the skirt-to-top
convention, so either feeds the formulas above. For single-diamond images the model box is ~3 px
higher than the VLM box, which is ~3 px of extra apparent height at 64 px (~5%): prefer the VLM
box when `refine.n_diamonds == 1`.

## 4. Where this lives

- `beacon_labeler/core/beacon_model.py`: CAD constants, camera, gravity, hull + bbox projection.
- `beacon_labeler/core/refine.py`: `refine_beacon`, `_slack_fit`, `_refit`.
- `beacon_labeler/config.yaml` `refine:`: all thresholds quoted above.
- Range from box: `NOTES.md` "Beacon localisation (`beacon_pose.py`)" > "Box localisation".
