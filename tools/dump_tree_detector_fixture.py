"""Dump Victoria Park scans + GA2 tree detections as a C++ test fixture.

Run with the GA2 Python environment:
    python tools/dump_tree_detector_fixture.py ../SensorFusion___GA2 tests/data/tree_detector_vp.txt

Output format (plain text):
    SCAN <index> <num_beams>
    <num_beams ranges in metres, as float32>
    TREES <m>
    <range> <bearing>      (m lines, GA2 extract_tree_measurements with max_range = 50)
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.io import loadmat

MAX_RANGE = 50.0
SCAN_INDICES = [2, 100, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                4500, 5000, 5500, 6000, 6500, 7000]


def main() -> None:
    ga2_root = Path(sys.argv[1]).resolve()
    out_path = Path(sys.argv[2])
    sys.path.insert(0, str(ga2_root / "src"))
    from graphslam.preprocessing import extract_tree_measurements  # noqa: E402

    raw = loadmat(str(ga2_root / "data" / "victoria_park" / "raw" / "aa3_lsr2"))
    scans = raw["LASER"] / 100.0  # cm -> m, as in GA2 VictoriaParkLoader

    with out_path.open("w") as f:
        for idx in SCAN_INDICES:
            # C++ LaserScan stores float32: run GA2 on exactly the same values.
            scan = scans[idx].astype(np.float32)
            z = extract_tree_measurements(scan.astype(np.float64), MAX_RANGE)
            f.write(f"SCAN {idx} {scan.size}\n")
            f.write(" ".join(f"{r:.9g}" for r in scan) + "\n")
            f.write(f"TREES {len(z)}\n")
            for r, b in z:
                f.write(f"{r:.17g} {b:.17g}\n")
    print(f"wrote {len(SCAN_INDICES)} scans to {out_path}")


if __name__ == "__main__":
    main()
