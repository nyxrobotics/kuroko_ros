#!/usr/bin/env bash
# Make all joint axes positive in a xacro file and swap/negate limits when flipped.
# Usage:
#   ./xacro_axis_to_positive.sh input.xacro output.xacro
#   ./xacro_axis_to_positive.sh -i input.xacro   # in-place edit

set -euo pipefail

usage() {
  echo "Usage: $0 [-i] <input.xacro> [output.xacro]" >&2
  exit 1
}

INPLACE=0
if [[ $# -lt 1 ]]; then usage; fi
if [[ "${1:-}" == "-i" ]]; then
  INPLACE=1
  shift || true
fi
if [[ $# -lt 1 ]]; then usage; fi

IN="$1"
OUT="${2:-}"

if [[ ! -f "$IN" ]]; then
  echo "Error: file not found: $IN" >&2
  exit 1
fi
if [[ $INPLACE -eq 0 && -z "$OUT" ]]; then
  usage
fi

python3 - <<'PY' "$IN" "$OUT" "$INPLACE"
import sys
from xml.etree import ElementTree as ET

in_path  = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None
inplace  = bool(int(sys.argv[3])) if len(sys.argv) > 3 else False

def tagname(t): return t.split('}')[-1]

def parse_xyz(s):
    """Parse the xyz attribute into a list of three floats."""
    try:
        p = [q for q in s.strip().replace(',', ' ').split()]
        if len(p) != 3:
            return None
        return [float(p[0]), float(p[1]), float(p[2])]
    except Exception:
        return None

def fmt_num(x):
    """Format a float compactly (avoid trailing zeros)."""
    if abs(x - round(x)) < 1e-12:
        return str(int(round(x)))
    return f"{x:.6f}".rstrip('0').rstrip('.')

def count_nonzero(v, eps=1e-12): 
    """Count non-zero components in a vector."""
    return sum(1 for x in v if abs(x) > eps)

tree = ET.parse(in_path)
root = tree.getroot()

changed = 0
warns = []

for joint in root.iter():
    if tagname(joint.tag) != 'joint':
        continue
    jname = joint.attrib.get('name', '(unnamed)')

    # Find <axis xyz="...">
    axis_elem = None
    for ch in joint:
        if tagname(ch.tag) == 'axis':
            axis_elem = ch
            break
    if axis_elem is None:
        continue
    xyz_attr = axis_elem.attrib.get('xyz')
    if not xyz_attr:
        continue

    vec = parse_xyz(xyz_attr)
    if vec is None:
        warns.append(f"[{jname}] axis not numeric or malformed: {xyz_attr!r} -> skipped")
        continue

    # Only handle single-axis joints
    if count_nonzero(vec) != 1:
        warns.append(f"[{jname}] axis has multiple non-zero components {vec} -> skipped")
        continue

    idx = next(i for i, x in enumerate(vec) if abs(x) > 1e-12)
    if vec[idx] < 0:
        # Flip axis to positive direction
        vec = [-v for v in vec]
        axis_elem.set('xyz', " ".join(fmt_num(x) for x in vec))

        # Find and update <limit> element if present
        limit_elem = None
        for ch in joint:
            if tagname(ch.tag) == 'limit':
                limit_elem = ch
                break
        if limit_elem is not None:
            lower = limit_elem.attrib.get('lower')
            upper = limit_elem.attrib.get('upper')
            def to_float(s):
                try: return float(s)
                except Exception: return None
            if lower is not None and upper is not None:
                lo = to_float(lower); up = to_float(upper)
                if lo is not None and up is not None:
                    # Swap and negate lower/upper limits
                    limit_elem.set('lower', fmt_num(-up))
                    limit_elem.set('upper', fmt_num(-lo))
                else:
                    warns.append(f"[{jname}] limit not numeric: lower={lower!r}, upper={upper!r} -> unchanged")
        changed += 1

xml_bytes = ET.tostring(root, encoding='utf-8')
decl = b'<?xml version="1.0"?>\n'
if not xml_bytes.lstrip().startswith(b'<?xml'):
    xml_bytes = decl + xml_bytes

if inplace:
    with open(in_path, 'wb') as f: f.write(xml_bytes)
    print(f"✅ Updated in-place: {in_path}")
else:
    with open(out_path, 'wb') as f: f.write(xml_bytes)
    print(f"✅ Wrote: {out_path}")
print(f"Modified joints: {changed}")
for w in warns:
    print(f"WARN: {w}", file=sys.stderr)
PY
