#!/usr/bin/env python3
"""Visualise the beacon model used by beacon_pose.py.

Draws, in the beacon frame (origin at base centre, Z up, metres):
  - the four diamonds exactly as BeaconModel stores them, keypoints coloured
    by role (bottom / left / top / right) and the face normals
  - the skirt: a 0.559 m square block from the base up to where the white
    section starts
  - the white section the diamonds sit on, from CAD (square frustum), placed
    vertically by fitting it to the diamond plane
  - the full-beacon envelope (0.559 x 0.559 x 0.52605 m) as a wireframe; the
    lit top above the white section is not modelled

The white section's height above the base isn't measured directly. It is where
the diamond plane reaches the CAD bottom half-width, and the figure reports how
far each diamond corner sits from the CAD face plane as a consistency check.

Usage:
    python3 voxl/visualize_beacon_mesh.py                  # interactive window
    python3 voxl/visualize_beacon_mesh.py --out mesh.png   # save instead
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import beacon_pose as bp  # noqa: E402

ENVELOPE_XY_M = 0.559      # [reported] full beacon footprint = skirt width
ENVELOPE_Z_M = 0.52605     # [reported] full beacon height
WHITE_BOTTOM_W_M = 0.432   # [CAD] white section, face width at its bottom
WHITE_TOP_W_M = 0.254      # [CAD] white section, face width at its top
WHITE_H_M = 0.291548       # [CAD] white section, vertical height

# Reference palette (dataviz skill): categorical slots 1-4 for the keypoint
# roles, ink tokens for everything else.
ROLE_COLOR = {"bottom": "#2a78d6", "left": "#eb6834", "top": "#1baf7a", "right": "#eda100"}
SURFACE, INK, INK_2, MUTED, HAIRLINE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#c3c2b7"
WHITE_FILL, SKIRT_FILL = "#f0efec", "#c3c2b7"


def model_faces_zup(model):
    """BeaconModel stores faces z-down as (x, -y, -z); undo that for display."""
    return model.faces * np.array([1.0, -1.0, -1.0])


def face_normal(quad):
    """Outward unit normal of a diamond (bottom, left, top, right)."""
    n = np.cross(quad[2] - quad[0], quad[3] - quad[1])
    if n[:2] @ quad.mean(axis=0)[:2] < 0:
        n = -n
    return n / np.linalg.norm(n)


def square_ring(half, z, u):
    """Square cross-section of half-width `half` at height z, faces normal to ±u."""
    side = np.cross([0, 0, 1.0], u)
    pts = [half * u + half * side, half * u - half * side,
           -half * u - half * side, -half * u + half * side]
    return np.array([[p[0], p[1], z] for p in pts])


def white_section(quads):
    """Place the CAD white-section frustum on the diamond plane.

    Returns (bottom_ring, top_ring, z0, corner_offsets) where z0 is the height
    at which the diamond plane reaches the CAD bottom half-width, and
    corner_offsets are the face-0 diamond corners' signed distances (m) from
    the CAD face plane.
    """
    q0 = quads[0]
    n, c = face_normal(q0), q0.mean(axis=0)
    u = np.r_[n[:2], 0] / np.linalg.norm(n[:2])            # horizontal outward dir
    # Diamond plane: n.(s*u + z*k - c) = 0  ->  z where s = CAD bottom half-width.
    hb, ht = WHITE_BOTTOM_W_M / 2, WHITE_TOP_W_M / 2
    z0 = (n @ c - hb * (n @ u)) / n[2]
    bottom, top = square_ring(hb, z0, u), square_ring(ht, z0 + WHITE_H_M, u)
    # CAD face plane through the face-0 bottom edge and top edge.
    side = np.cross([0, 0, 1.0], u)
    p_bot, p_top = hb * u + np.r_[0, 0, z0], ht * u + np.r_[0, 0, z0 + WHITE_H_M]
    m = np.cross(side, p_top - p_bot)
    m /= np.linalg.norm(m)
    if m @ u < 0:
        m = -m
    return bottom, top, z0, (q0 - p_bot) @ m


def prism_faces(lower, upper):
    sides = [[lower[i], lower[(i + 1) % 4], upper[(i + 1) % 4], upper[i]] for i in range(4)]
    return sides + [list(lower), list(upper)]


def box_edges(half, h):
    xs, ys = (-half, half), (-half, half)
    pts = np.array([[x, y, z] for x in xs for y in ys for z in (0.0, h)])
    edges = []
    for i in range(8):
        for j in range(i + 1, 8):
            if np.sum(np.abs(pts[i] - pts[j]) > 1e-9) == 1:
                edges.append((pts[i], pts[j]))
    return edges


def draw(ax, quads, white, skirt, envelope, title, elev, azim, label_face0,
         ortho=False, hide=None):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    ax.add_collection3d(Poly3DCollection(prism_faces(*skirt), facecolor=SKIRT_FILL,
                                         edgecolor=MUTED, linewidth=0.8, alpha=0.18))
    ax.add_collection3d(Poly3DCollection(prism_faces(*white), facecolor=WHITE_FILL,
                                         edgecolor=HAIRLINE, linewidth=0.8, alpha=0.4))
    for a, b in envelope:
        ax.plot(*zip(a, b), color=MUTED, linewidth=1.0, linestyle=(0, (4, 3)))

    for k, q in enumerate(quads):
        ax.add_collection3d(Poly3DCollection([list(q)], facecolor=INK, edgecolor=INK,
                                             linewidth=0.5, alpha=0.9))
        for name, p in zip(bp.KPT_NAMES, q):
            ax.scatter(*p, s=36, color=ROLE_COLOR[name], edgecolors=SURFACE,
                       linewidths=1.5, depthshade=False, zorder=5)
        c, n = q.mean(axis=0), face_normal(q)
        ax.quiver(*c, *(0.12 * n), color=INK_2, linewidth=1.2, arrow_length_ratio=0.25)
        if label_face0 and k == 0:
            for name, p in zip(bp.KPT_NAMES, q):
                d = (p - c) / np.linalg.norm(p - c)
                ax.text(*(p + 0.045 * d + np.array([0, -0.05, 0])), name, color=INK,
                        fontsize=8, ha="center", va="center")

    for v in np.eye(3) * 0.15:
        ax.quiver(0, 0, 0, *v, color=MUTED, linewidth=1, arrow_length_ratio=0.2)
    for v, s in (((0.16, 0, 0), "X"), ((0, 0.16, 0), "Y"), ((0, 0, 0.16), "Z")):
        ax.text(*v, s, color=MUTED, fontsize=7)

    r = ENVELOPE_XY_M / 2 + 0.05
    ax.set_xlim(-r, r)
    ax.set_ylim(-r, r)
    ax.set_zlim(0, ENVELOPE_Z_M + 0.05)
    ax.set_box_aspect((2 * r, 2 * r, ENVELOPE_Z_M + 0.05))
    ax.view_init(elev=elev, azim=azim)
    if ortho:
        ax.set_proj_type("ortho")
    ax.set_title(title, color=INK, fontsize=10)
    ax.set_xlabel("x (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("y (m)", color=MUTED, fontsize=7)
    ax.set_zlabel("z (m)", color=MUTED, fontsize=7)
    ax.tick_params(colors=MUTED, labelsize=6)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((1, 1, 1, 0))
        axis._axinfo["grid"]["color"] = "#e1e0d9"
    if hide:
        getattr(ax, f"set_{hide}ticks")([])
        getattr(ax, f"set_{hide}label")("")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", help="save to this image file instead of opening a window")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    import matplotlib
    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    model = bp.BeaconModel()
    quads = model_faces_zup(model)
    w_bottom, w_top, z0, offsets = white_section(quads)
    u = np.r_[face_normal(quads[0])[:2], 0]
    u /= np.linalg.norm(u)
    half = ENVELOPE_XY_M / 2
    skirt = (square_ring(half, 0.0, u), square_ring(half, z0, u))
    envelope = box_edges(half, ENVELOPE_Z_M)
    n0 = face_normal(quads[0])
    tilt_diamond = math.degrees(math.asin(n0[2]))
    tilt_cad = math.degrees(math.atan((WHITE_BOTTOM_W_M - WHITE_TOP_W_M) / 2 / WHITE_H_M))
    diag_tb = np.linalg.norm(quads[0][2] - quads[0][0])
    diag_lr = np.linalg.norm(quads[0][3] - quads[0][1])

    fig = plt.figure(figsize=(15, 5.8), facecolor=SURFACE)
    views = [("Isometric", 22, -60, True, False, None),
             ("Front elevation (face 0, orthographic)", 0, -90, True, True, "y"),
             ("Top-down (orthographic)", 90, -90, False, True, "z")]
    for i, (title, elev, azim, lab, ortho, hide) in enumerate(views):
        ax = fig.add_subplot(1, 3, i + 1, projection="3d", facecolor=SURFACE)
        draw(ax, quads, (w_bottom, w_top), skirt, envelope, title, elev, azim, lab, ortho, hide)

    fig.suptitle("Beacon model used by beacon_pose.py", color=INK, fontsize=13, y=0.985)
    stats = (f"diamond diagonals {diag_tb:.4f} / {diag_lr:.4f} m   centre {model.center_height:.4f} m "
             f"above base   face tilt from vertical: diamonds {tilt_diamond:.2f}°, CAD {tilt_cad:.2f}°\n"
             f"white section (CAD) z = {z0:.4f} to {z0 + WHITE_H_M:.4f} m   skirt height "
             f"{z0:.4f} m (derived)   top above white section {ENVELOPE_Z_M - z0 - WHITE_H_M:.4f} m "
             f"(not modelled)   diamond corners off CAD face: max {1000 * np.abs(offsets).max():.1f} mm")
    fig.text(0.5, 0.03, stats, ha="center", color=INK_2, fontsize=9)

    handles = [Line2D([], [], marker="o", linestyle="", markersize=7,
                      markerfacecolor=ROLE_COLOR[n], markeredgecolor=SURFACE, label=n)
               for n in bp.KPT_NAMES]
    handles += [Line2D([], [], color=INK, linewidth=6, label="diamond"),
                Line2D([], [], color=WHITE_FILL, linewidth=6, label="white section (CAD)"),
                Line2D([], [], color=SKIRT_FILL, linewidth=6, label="skirt"),
                Line2D([], [], color=MUTED, linestyle=(0, (4, 3)), label="full envelope"),
                Line2D([], [], color=INK_2, label="face normal")]
    fig.legend(handles=handles, loc="upper center", ncol=9, frameon=False,
               bbox_to_anchor=(0.5, 0.945), fontsize=8, labelcolor=INK)
    fig.subplots_adjust(left=0.01, right=0.99, top=0.83, bottom=0.11, wspace=0.02)

    if args.out:
        fig.savefig(args.out, dpi=args.dpi, facecolor=SURFACE)
        print(f"saved {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
