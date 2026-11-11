#!/usr/bin/env python3
"""
review_boxes.py -- review / fix the beacon boxes from prelabel_beacons.py.

    py -3.10 training/review_boxes.py <prelabel_dir> -o <reviewed_dir>

<prelabel_dir> (images/ + labels/) is only READ. Every frame you leave is saved
to <reviewed_dir>/images + labels (YOLO "0 cx cy w h"), which is what
build_dataset.py takes. Stop any time (Q); run the same command again to carry
on -- frames already in <reviewed_dir> are loaded from there, and it starts at
the first frame not reviewed yet.

Mouse
    left-drag        draw a beacon box
    right-click      delete the box under the pointer (the smallest one)
Keys
    -> / D / Space   save, next frame          <- / A   save, previous frame
    1                water band, full width (default view)
    2 / 3            left / right half of the water band, zoomed
    F                whole frame
    U                undo the last change on this frame
    C                clear all boxes on this frame
    R                reset this frame to the pre-label boxes
    X                exclude this frame (unsure / unusable): not used for training
    Q / Esc          save and quit

What to do on each frame: box EVERY visible beacon (base to tip), delete boxes
that aren't beacons (platforms, signs, boats, shore), keep beacon-free frames
with no boxes. If you can't tell whether something is a beacon, press X.
"""

import argparse
import shutil
import sys
from pathlib import Path

import cv2

KEY_LEFT = {2424832, 65361, ord("a"), ord("A")}
KEY_RIGHT = {2555904, 65363, ord("d"), ord("D"), ord(" ")}
WIN = "review boxes"
MAX_VIEW_H = 860   # px: tallest image shown, so the window fits a laptop screen


def load_boxes(path: Path, w: int, h: int):
    """YOLO lines -> [(x1, y1, x2, y2)] pixels (class 0 only; others kept as raw lines)."""
    boxes, other = [], []
    if path.exists():
        for line in path.read_text().splitlines():
            v = line.split()
            if not v:
                continue
            if v[0] == "0" and len(v) >= 5:
                cx, cy, bw, bh = (float(x) for x in v[1:5])
                boxes.append((int(round((cx - bw / 2) * w)), int(round((cy - bh / 2) * h)),
                              int(round((cx + bw / 2) * w)), int(round((cy + bh / 2) * h))))
            else:
                other.append(line)
    return boxes, other


def boxes_to_lines(boxes, w, h):
    return [f"0 {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} {(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}"
            for x1, y1, x2, y2 in boxes]


class Reviewer:
    def __init__(self, src: Path, out: Path, roi_top: float, screen_w: int):
        self.src, self.out = src, out
        self.files = sorted(p for p in (src / "images").iterdir()
                            if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        if not self.files:
            sys.exit(f"no images in {src / 'images'}")
        (out / "images").mkdir(parents=True, exist_ok=True)
        (out / "labels").mkdir(parents=True, exist_ok=True)
        self.excl_path = out / "excluded.txt"
        self.excluded = set(self.excl_path.read_text().split()) if self.excl_path.exists() else set()
        self.roi_top, self.screen_w = roi_top, screen_w
        self.view = "1"
        self.i = next((k for k, p in enumerate(self.files) if not self.done(p)), 0)
        self.drag = None
        self.load()

    def done(self, p: Path) -> bool:
        return p.stem in self.excluded or (self.out / "labels" / f"{p.stem}.txt").exists()

    # ── frame load / save ──
    def load(self):
        p = self.files[self.i]
        self.img = cv2.imread(str(p))
        self.h, self.w = self.img.shape[:2]
        rev = self.out / "labels" / f"{p.stem}.txt"
        self.from_reviewed = rev.exists()
        self.boxes, self.other = load_boxes(rev if rev.exists() else self.src / "labels" / f"{p.stem}.txt",
                                            self.w, self.h)
        self.history = []

    def save(self):
        p = self.files[self.i]
        if p.stem in self.excluded:
            return
        dst = self.out / "images" / p.name
        if not dst.exists():
            shutil.copy2(p, dst)
        lines = boxes_to_lines(self.boxes, self.w, self.h) + self.other
        (self.out / "labels" / f"{p.stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))

    def exclude(self):
        p = self.files[self.i]
        self.excluded.add(p.stem)
        self.excl_path.write_text("\n".join(sorted(self.excluded)) + "\n")
        for f in (self.out / "images" / p.name, self.out / "labels" / f"{p.stem}.txt"):
            if f.exists():
                f.unlink()   # only the copies this tool made in <reviewed_dir>

    def change(self, new_boxes):
        self.history.append(list(self.boxes))
        self.boxes = new_boxes
        self.excluded.discard(self.files[self.i].stem)
        if self.excl_path.exists():
            self.excl_path.write_text("\n".join(sorted(self.excluded)) + ("\n" if self.excluded else ""))

    # ── view geometry: (x0, y0, x1, y1) crop of the frame, and its display scale ──
    def crop(self):
        y_roi = int(self.h * self.roi_top)
        if self.view == "f":
            c = (0, 0, self.w, self.h)
        elif self.view == "2":
            c = (0, y_roi, self.w // 2, self.h)
        elif self.view == "3":
            c = (self.w // 2, y_roi, self.w, self.h)
        else:
            c = (0, y_roi, self.w, self.h)
        s = self.screen_w / (c[2] - c[0])
        return c, min(s, MAX_VIEW_H / (c[3] - c[1]))   # keep it on a laptop screen

    def to_frame(self, vx, vy):
        (x0, y0, _, _), s = self.crop()
        return int(round(x0 + vx / s)), int(round(y0 + vy / s))

    def render(self):
        (x0, y0, x1, y1), s = self.crop()
        vis = cv2.resize(self.img[y0:y1, x0:x1], None, fx=s, fy=s,
                         interpolation=cv2.INTER_NEAREST if s >= 2 else cv2.INTER_LINEAR)
        for bx1, by1, bx2, by2 in self.boxes:
            cv2.rectangle(vis, (int((bx1 - x0) * s), int((by1 - y0) * s)),
                          (int((bx2 - x0) * s), int((by2 - y0) * s)), (0, 0, 255), 1 if s < 2 else 2)
        if self.drag and self.drag[2] is not None:
            (ax, ay), (bx, by) = self.drag[:2], self.drag[2]
            cv2.rectangle(vis, (ax, ay), (bx, by), (0, 255, 0), 1)
        p = self.files[self.i]
        state = ("EXCLUDED" if p.stem in self.excluded else
                 "reviewed" if self.from_reviewed else "pre-label")
        n_done = sum(self.done(f) for f in self.files)
        top = f"{self.i + 1}/{len(self.files)}  {p.name}  boxes {len(self.boxes)}  [{state}]  done {n_done}/{len(self.files)}"
        bar = cv2.copyMakeBorder(vis, 44, 0, 0, 0, cv2.BORDER_CONSTANT, value=(40, 40, 40))
        cv2.putText(bar, top, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, "drag=add  right-click=delete  <-/->=prev/next  1/2/3/F=view  U=undo  C=clear  "
                         "R=reset  X=exclude  Q=quit", (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1, cv2.LINE_AA)
        cv2.imshow(WIN, bar)

    # ── mouse ──
    def on_mouse(self, event, x, y, flags, _):
        y -= 44   # header bar
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag = [x, y, None]
        elif event == cv2.EVENT_MOUSEMOVE and self.drag:
            self.drag[2] = (x, y)
            self.render()
        elif event == cv2.EVENT_LBUTTONUP and self.drag:
            ax, ay = self.drag[:2]
            self.drag = None
            (fx1, fy1), (fx2, fy2) = self.to_frame(min(ax, x), min(ay, y)), self.to_frame(max(ax, x), max(ay, y))
            fx1, fy1 = max(0, fx1), max(0, fy1)
            fx2, fy2 = min(self.w, fx2), min(self.h, fy2)
            if fx2 - fx1 >= 3 and fy2 - fy1 >= 3:
                self.change(self.boxes + [(fx1, fy1, fx2, fy2)])
            self.render()
        elif event == cv2.EVENT_RBUTTONDOWN:
            fx, fy = self.to_frame(x, y)
            hit = [b for b in self.boxes if b[0] - 2 <= fx <= b[2] + 2 and b[1] - 2 <= fy <= b[3] + 2]
            if hit:
                smallest = min(hit, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
                self.change([b for b in self.boxes if b != smallest])
                self.render()

    # ── main loop ──
    def run(self):
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN, self.on_mouse)
        self.render()
        while True:
            k = cv2.waitKeyEx(50)
            if k == -1:
                if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:   # window closed
                    self.save()
                    break
                continue
            ch = k & 0xFF
            if k in KEY_RIGHT or k in KEY_LEFT:
                self.save()
                self.i = min(len(self.files) - 1, self.i + 1) if k in KEY_RIGHT else max(0, self.i - 1)
                self.load()
            elif ch in (ord("q"), ord("Q"), 27):
                self.save()
                break
            elif ch in (ord("1"), ord("2"), ord("3")):
                self.view = chr(ch)
            elif ch in (ord("f"), ord("F")):
                self.view = "f"
            elif ch in (ord("u"), ord("U")) and self.history:
                self.boxes = self.history.pop()
            elif ch in (ord("c"), ord("C")):
                self.change([])
            elif ch in (ord("r"), ord("R")):
                self.change(load_boxes(self.src / "labels" / f"{self.files[self.i].stem}.txt",
                                       self.w, self.h)[0])
            elif ch in (ord("x"), ord("X")):
                self.exclude()
                self.i = min(len(self.files) - 1, self.i + 1)
                self.load()
            self.render()
        cv2.destroyAllWindows()
        n_rev = len(list((self.out / "labels").glob("*.txt")))
        print(f"{n_rev} frames reviewed, {len(self.excluded)} excluded -> {self.out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prelabel", help="prelabel_beacons.py output (images/ + labels/)")
    ap.add_argument("-o", "--out", required=True, help="reviewed folder (created; reused to resume)")
    ap.add_argument("--roi-top", type=float, default=0.45, help="water band starts at this fraction of the height")
    ap.add_argument("--width", type=int, default=1600, help="window image width (px)")
    a = ap.parse_args()
    src, out = Path(a.prelabel), Path(a.out)
    if src.resolve() == out.resolve():
        sys.exit("the reviewed folder must be different from the pre-label folder")
    Reviewer(src, out, a.roi_top, a.width).run()


if __name__ == "__main__":
    main()
