#!/usr/bin/env python3
"""Build A/C marker-observation hybrids for controlled orientation diagnostics.

The datasets must share cameras, trajectory, marker positions, and frame indices.
H01/H03/H06 always come from C. H02/H04/H05 are selected independently from
the A (original orientation) or C (small-angle orientation) detections.
"""
from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import numpy as np

from evaluate_dataset_metrics import (
    dump_json,
    evaluate_observations,
    load_json,
    projection_from_camera,
)


TUNED_MARKERS = ("H02", "H04", "H05")


def matched_map(frame, camera_id):
    return {
        item["marker_id"]: item["pixel_uv"]
        for item in frame["cameras"][camera_id]["matched_by_gt_for_evaluation"]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-a", type=Path, required=True)
    parser.add_argument("--dataset-c", type=Path, required=True)
    parser.add_argument("--detections-a", type=Path, required=True)
    parser.add_argument("--detections-c", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    camera_data = load_json(args.dataset_c / "camera.json")
    cameras = {item["camera_id"]: item for item in camera_data["cameras"]}
    projections = {
        camera_id: projection_from_camera(camera)
        for camera_id, camera in cameras.items()
    }
    gt3d_frames = load_json(args.dataset_c / "gt_3d_frames.json")["frames"]
    pose_frames = load_json(args.dataset_c / "object_pose_frames.json")["frames"]
    scene = load_json(args.dataset_c / "scene_config.json")
    rigid_body = next(
        body
        for body in load_json(args.dataset_c / "rigid_bodies.json")["rigid_bodies"]
        if body["rigid_body_id"] == scene["rigid_body_id"]
    )
    local_points = {
        marker["id"]: np.asarray(marker["position_cm"], dtype=float)
        for marker in rigid_body["markers"]
    }
    frames_a = load_json(args.detections_a)["frames"]
    frames_c = load_json(args.detections_c)["frames"]
    if len(frames_a) != len(frames_c):
        raise ValueError("A and C detection frame counts differ")

    summary = []
    for sources in itertools.product("AC", repeat=len(TUNED_MARKERS)):
        source_by_marker = dict(zip(TUNED_MARKERS, sources))
        name = "_".join(
            f"{marker}{source_by_marker[marker]}" for marker in TUNED_MARKERS
        )
        detection_frames = []
        observations = []
        for frame_a, frame_c in zip(frames_a, frames_c):
            if frame_a["frame_index"] != frame_c["frame_index"]:
                raise ValueError("A and C frame indices differ")
            output_frame = {
                "frame_index": frame_c["frame_index"],
                "cameras": {},
            }
            frame_observations = {}
            for camera_id in cameras:
                map_a = matched_map(frame_a, camera_id)
                map_c = matched_map(frame_c, camera_id)
                selected = {}
                for marker_id in local_points:
                    source = source_by_marker.get(marker_id, "C")
                    source_map = map_a if source == "A" else map_c
                    if marker_id in source_map:
                        selected[marker_id] = source_map[marker_id]
                frame_observations[camera_id] = selected
                output_frame["cameras"][camera_id] = {
                    "detections": [],
                    "matched_by_gt_for_evaluation": [
                        {"marker_id": marker_id, "pixel_uv": pixel}
                        for marker_id, pixel in sorted(selected.items())
                    ],
                    "unmatched_detection_indices": [],
                }
            observations.append(frame_observations)
            detection_frames.append(output_frame)

        metrics, reconstructed, poses = evaluate_observations(
            f"A/C hybrid {name}",
            observations,
            projections,
            gt3d_frames,
            pose_frames,
            local_points,
        )
        output = args.output / name / "base_pipeline"
        dump_json(output / "detections_2d.json", {"frames": detection_frames})
        dump_json(
            output / "reconstructed_markers_3d.json",
            {"image_pipeline": reconstructed},
        )
        dump_json(
            output / "estimated_object_pose.json",
            {"image_pipeline": poses},
        )
        dump_json(
            args.output / name / "hybrid_base_metrics.json",
            {
                "name": name,
                "source_by_marker": source_by_marker,
                "multicamera": metrics,
            },
        )
        summary.append(
            {
                "name": name,
                "source_by_marker": source_by_marker,
                "observation_count": sum(
                    len(marker_map)
                    for frame in observations
                    for marker_map in frame.values()
                ),
                "position_error_mm": metrics["position_accuracy"]["error_mm"],
                "orientation_error_deg": metrics["orientation_accuracy"][
                    "geodesic_error_deg"
                ],
                "marker_3d_error_mm": metrics["marker_3d_error_mm"],
            }
        )

    dump_json(args.output / "hybrid_base_summary.json", {"variants": summary})


if __name__ == "__main__":
    main()
