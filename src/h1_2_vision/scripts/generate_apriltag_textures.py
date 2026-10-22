#!/usr/bin/python3
"""Reproducibly render the exact tag36h11 textures bound by tag_map.yaml."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np


TAG_IDS = (10, 11, 12, 20, 21, 22, 23)
INNER_PIXELS = 800
QUIET_ZONE_PIXELS = 100


def render(tag_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.drawMarker(dictionary, tag_id, INNER_PIXELS)
    canvas = np.full(
        (INNER_PIXELS + 2 * QUIET_ZONE_PIXELS,) * 2, 255, dtype=np.uint8)
    canvas[
        QUIET_ZONE_PIXELS:QUIET_ZONE_PIXELS + INNER_PIXELS,
        QUIET_ZONE_PIXELS:QUIET_ZONE_PIXELS + INNER_PIXELS,
    ] = marker
    return canvas


def write_checked(path: Path, image: np.ndarray) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_PNG_COMPRESSION, 9]):
        raise RuntimeError(f'could not write {path}')
    payload = path.read_bytes()
    decoded = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if decoded is None or not np.array_equal(decoded, image):
        raise RuntimeError(f'lossless texture verification failed for {path}')
    return hashlib.sha256(payload).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    description = workspace / 'src' / 'h1_2_description' / 'models'
    destinations = {
        10: description / 'incubator/materials/textures/tag36h11_10.png',
        11: description / 'incubator/materials/textures/tag36h11_11.png',
        12: description / 'incubator/materials/textures/tag36h11_12.png',
        20: description / 'green_cube/materials/textures/tag36h11_20.png',
        21: description / 'green_cube/materials/textures/tag36h11_21.png',
        22: description / 'green_cube/materials/textures/tag36h11_22.png',
        23: description / 'green_cube/materials/textures/tag36h11_23.png',
    }
    records = []
    for tag_id in TAG_IDS:
        path = destinations[tag_id]
        digest = write_checked(path, render(tag_id))
        records.append(f'{digest}  {path.relative_to(workspace)}')
    manifest = workspace / 'src/h1_2_vision/config/tag_textures.sha256'
    manifest.write_text('\n'.join(records) + '\n', encoding='ascii')


if __name__ == '__main__':
    main()
