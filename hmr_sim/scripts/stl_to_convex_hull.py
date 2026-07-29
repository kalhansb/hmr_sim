#!/usr/bin/env python3
"""Compute convex hull of an STL mesh and save it as another STL.

Usage:
    stl_to_convex_hull.py <input.stl> <output.stl>
    stl_to_convex_hull.py <input_dir> <output_dir>  # batch all *.STL in directory
"""
import argparse
import sys
from pathlib import Path

import trimesh


def convert(in_path: Path, out_path: Path) -> None:
    mesh = trimesh.load(str(in_path), force='mesh')
    hull = mesh.convex_hull
    out_path.parent.mkdir(parents=True, exist_ok=True)
    hull.export(str(out_path))
    print(f'{in_path.name}: {len(mesh.faces)} → {len(hull.faces)} faces  →  {out_path}')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path, help='input STL file or directory')
    p.add_argument('output', type=Path, help='output STL file or directory')
    args = p.parse_args()

    if args.input.is_dir():
        args.output.mkdir(parents=True, exist_ok=True)
        stls = sorted(args.input.glob('*.STL')) + sorted(args.input.glob('*.stl'))
        if not stls:
            print(f'No STL files in {args.input}', file=sys.stderr)
            return 1
        for f in stls:
            convert(f, args.output / f.name)
    else:
        convert(args.input, args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
