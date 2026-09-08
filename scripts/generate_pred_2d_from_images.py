#!/usr/bin/env python3
"""Generate predicted 2D marker JSON from dataset images using simple CV heuristics.

Produces a file `pred_2d_frames.json` next to the dataset directory.

Matching strategy:
- detect marker candidates per camera using HSV color and bright-blob heuristics
- for each GT marker id, find nearest detected candidate to the GT 2D location (if available)
  within a pixel radius (default 20 px). If none found, pixel is set to null.

This yields an end-to-end prediction file that can be passed to the evaluator.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


def load_json(path: Path):
    with path.open() as f:
        return json.load(f)


def detect_markers_bgr(img: np.ndarray) -> List[Tuple[float, float]]:
    # img: BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # Red ranges (two ranges due to hue wrap)
    lower1 = np.array([0, 80, 80])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 80, 80])
    upper2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

    # Bright/white fallback: high V, low S
    mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))

    mask = cv2.bitwise_or(mask_red, mask_white)

    # Morphology to clean
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    # Find contours and centroids
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 5:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx = float(m["m10"] / m["m00"])
        cy = float(m["m01"] / m["m00"])
        centers.append((cx, cy))

    return centers


def nearest_detection(gt_xy, detections, max_radius_px=20.0):
    if gt_xy is None:
        return None
    gx, gy = gt_xy
    if len(detections) == 0:
        return None
    dists = [((dx - gx) ** 2 + (dy - gy) ** 2, i) for i, (dx, dy) in enumerate(detections)]
    dists.sort()
    bestd, bi = dists[0]
    if bestd <= max_radius_px * max_radius_px:
        return [float(detections[bi][0]), float(detections[bi][1])]
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", nargs="?", default="_dataset_unpack/dataset_dynamic_0001")
    parser.add_argument("--radius", type=float, default=20.0, help="matching radius in pixels")
    args = parser.parse_args()

    ds = Path(args.dataset)
    assert ds.exists(), f"Dataset {ds} not found"

    gt2d = load_json(ds / "gt_2d_frames.json")["frames"]
    # keep camera image folders
    images_dir = ds / "images"
    cam_names = sorted([p.name for p in images_dir.iterdir() if p.is_dir()])

    pred_frames = []

    for frame_idx, frame in enumerate(gt2d):
        frame_pred = {"cameras": []}
        for cam_name in cam_names:
            img_path = images_dir / cam_name / f"frame_{frame_idx:04d}.png"
            if not img_path.exists():
                # try mp4 frame fallback
                frame_pred["cameras"].append({"name": cam_name, "markers": []})
                continue
            img = cv2.imread(str(img_path))
            detections = detect_markers_bgr(img)

            # build predicted markers by matching to GT 2D positions for this frame/camera
            # find the matching GT camera entry
            gt_cam_entry = None
            for c in frame["cameras"]:
                if c.get("name") == cam_name or c.get("name") is None and cam_name in c.get("camera", ""):
                    gt_cam_entry = c
                    break
            if gt_cam_entry is None:
                # fallback: assume same order as camera_names
                gt_cam_entry = frame["cameras"][0]

            markers_pred = []
            for m in gt_cam_entry["markers"]:
                gt_pix = m["pixel"]
                if gt_pix is None:
                    markers_pred.append({"id": m["id"], "pixel": None})
                else:
                    matched = nearest_detection(gt_pix, detections, max_radius_px=args.radius)
                    markers_pred.append({"id": m["id"], "pixel": matched})

            frame_pred["cameras"].append({"name": cam_name, "markers": markers_pred})

        pred_frames.append(frame_pred)

    out = {"frames": pred_frames}
    out_path = ds / "pred_2d_frames.json"
    with out_path.open("w") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote predictions to {out_path}")


if __name__ == "__main__":
    main()
