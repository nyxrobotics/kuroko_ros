#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import xml.etree.ElementTree as ET

EPS = 1e-8

def parse_vec3(s: str):
    vals = [float(x) for x in s.strip().split()]
    if len(vals) != 3:
        raise ValueError("xyz must have 3 components")
    return vals

def format_vec3(v):
    return f"{v[0]:.8g} {v[1]:.8g} {v[2]:.8g}"

def should_flip_axis(xyz):
    """Flip if axis is essentially along a principal axis and the dominant component is negative."""
    ax = [float(x) for x in xyz]
    # zero vector? skip
    if abs(ax[0]) < EPS and abs(ax[1]) < EPS and abs(ax[2]) < EPS:
        return False
    # find dominant component
    idx = max(range(3), key=lambda i: abs(ax[i]))
    # others must be ~0 to be considered principal
    others = [ax[i] for i in range(3) if i != idx]
    if all(abs(o) < 1e-6 for o in others) and ax[idx] < 0.0:
        return True
    return False

def flip_limits(limit_elem):
    """Swap & negate lower/upper if present."""
    if limit_elem is None:
        return
    lower_e = None
    upper_e = None
    for child in limit_elem:
        tag = child.tag.split('}')[-1]  # strip namespace if any
        if tag == "lower":
            lower_e = child
        elif tag == "upper":
            upper_e = child
    # read
    def readf(e):
        if e is None or e.text is None or e.text.strip() == "":
            return None
        try:
            return float(e.text.strip())
        except Exception:
            return None
    lower = readf(lower_e)
    upper = readf(upper_e)
    # compute flipped values if both exist
    if lower is not None and upper is not None:
        new_lower = -upper
        new_upper = -lower
        lower_e.text = f"{new_lower:.8g}"
        upper_e.text = f"{new_upper:.8g}"
    elif lower is not None:
        # only lower: becomes upper after flip
        upper_val = -lower
        if upper_e is None:
            upper_e = ET.SubElement(limit_elem, "upper")
        upper_e.text = f"{upper_val:.8g}"
        # optional: remove/clear lower
        lower_e.text = ""
    elif upper is not None:
        # only upper: becomes lower after flip
        lower_val = -upper
        if lower_e is None:
            lower_e = ET.SubElement(limit_elem, "lower")
        lower_e.text = f"{lower_val:.8g}"
        upper_e.text = ""

def process_axis(axis_elem):
    """Flip axis.xyz and limits if needed."""
    if axis_elem is None:
        return False
    # find <xyz>
    xyz_elem = None
    limit_elem = None
    for child in axis_elem:
        tag = child.tag.split('}')[-1]
        if tag == "xyz":
            xyz_elem = child
        elif tag == "limit":
            limit_elem = child
    if xyz_elem is None or xyz_elem.text is None:
        return False
    try:
        v = parse_vec3(xyz_elem.text)
    except Exception:
        return False
    if should_flip_axis(v):
        v = [-v[0], -v[1], -v[2]]
        # normalize (SDF expects unit vector)
        n = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if n > EPS:
            v = [vi / n for vi in v]
        xyz_elem.text = format_vec3(v)
        flip_limits(limit_elem)
        return True
    return False

def fix_sdf_axes(path_in, path_out=None):
    tree = ET.parse(path_in)
    root = tree.getroot()
    flipped = 0
    for joint in root.iter():
        if joint.tag.endswith("joint"):
            # primary axis
            a1 = None
            a2 = None
            for child in joint:
                tag = child.tag.split('}')[-1]
                if tag == "axis" and a1 is None:
                    a1 = child
                elif tag == "axis2" and a2 is None:
                    a2 = child
            if process_axis(a1):
                flipped += 1
            if process_axis(a2):
                flipped += 1
    out = path_out or path_in
    tree.write(out, encoding="utf-8", xml_declaration=True)
    return flipped, out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="Input SDF file")
    ap.add_argument("--out", dest="outp", required=False, help="Output SDF file (default: overwrite input)")
    args = ap.parse_args()
    cnt, outp = fix_sdf_axes(args.inp, args.outp)
    print(f"[OK] Flipped {cnt} joint axis/limits. Wrote: {outp}")

if __name__ == "__main__":
    main()

