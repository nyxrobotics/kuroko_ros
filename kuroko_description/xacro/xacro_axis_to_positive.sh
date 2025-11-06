#!/usr/bin/env bash
# Make joint axes positive in xacro files; when flipped, swap/negate <limit lower/upper>.
# Preserves XML namespaces (keeps 'xacro:'), supports wildcards, adds verbose and dry-run.
# Usage:
#   ./xacro_axis_to_positive.sh -i [-v] *.xacro
#   ./xacro_axis_to_positive.sh [-v] [-n] file1.xacro file2.xacro

set -euo pipefail

usage() {
  echo "Usage: $0 [-i] [-n] [-v] <file1.xacro> [file2.xacro ...]" >&2
  exit 1
}

INPLACE=0
DRYRUN=0
VERBOSE=0
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i) INPLACE=1; shift ;;
    -n) DRYRUN=1; shift ;;
    -v) VERBOSE=1; shift ;;
    -h|--help) usage ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
[[ ${#ARGS[@]} -gt 0 ]] || usage

for IN in "${ARGS[@]}"; do
  [[ -f "$IN" ]] || { echo "Skipping: $IN (not found)" >&2; continue; }

  OUT=""
  if [[ $INPLACE -eq 0 ]]; then
    base="${IN%.*}"; OUT="${base}_positive.xacro"
  fi

  echo "==> Processing: $IN"
  python3 - "$IN" "${OUT}" "${INPLACE}" "${DRYRUN}" "${VERBOSE}" <<'PY'
import sys
from xml.etree import ElementTree as ET
in_path  = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None
inplace  = bool(int(sys.argv[3])) if len(sys.argv) > 3 else False
dryrun   = bool(int(sys.argv[4])) if len(sys.argv) > 4 else False
verbose  = bool(int(sys.argv[5])) if len(sys.argv) > 5 else False

def tagname(t): return t.split('}')[-1]
def parse_xyz(s):
    try:
        # tolerate commas / extra spaces
        p = [q for q in s.strip().replace(',', ' ').split()]
        if len(p) != 3: return None
        return [float(p[0]), float(p[1]), float(p[2])]
    except Exception:
        return None
def fmt_num(x):
    if abs(x - round(x)) < 1e-12: return str(int(round(x)))
    return f"{x:.6f}".rstrip('0').rstrip('.')
def count_nonzero(v, eps=1e-12): return sum(1 for x in v if abs(x) > eps)

# Preserve original prefixes to avoid ns0:
nsmap = {}
it = ET.iterparse(in_path, events=('start','start-ns'))
for ev, obj in it:
    if ev == 'start-ns':
        prefix, uri = obj
        nsmap[prefix or ''] = uri
root = it.root
for prefix, uri in nsmap.items():
    ET.register_namespace(prefix, uri)

changed, skipped = 0, 0

for joint in root.iter():
    if tagname(joint.tag) != 'joint':
        continue
    jname = joint.attrib.get('name', '(unnamed)')
    axis_elem = next((c for c in joint if tagname(c.tag) == 'axis'), None)
    if axis_elem is None:
        if verbose: print(f"  - {jname}: no <axis>", file=sys.stderr)
        continue
    xyz_attr = axis_elem.attrib.get('xyz')
    if not xyz_attr:
        if verbose: print(f"  - {jname}: empty axis xyz", file=sys.stderr)
        continue

    vec = parse_xyz(xyz_attr)
    if vec is None:
        if verbose: print(f"  - {jname}: axis not numeric -> {xyz_attr!r}", file=sys.stderr)
        skipped += 1
        continue

    if count_nonzero(vec) != 1:
        if verbose: print(f"  - {jname}: not single-axis {vec}", file=sys.stderr)
        skipped += 1
        continue

    idx = next(i for i, x in enumerate(vec) if abs(x) > 1e-12)
    if verbose:
        print(f"  . {jname}: axis {vec} (nonzero at {['x','y','z'][idx]})")

    if vec[idx] < 0:
        old_axis = " ".join(fmt_num(x) for x in vec)
        vec = [-v for v in vec]
        new_axis = " ".join(fmt_num(x) for x in vec)
        if not dryrun:
            axis_elem.set('xyz', new_axis)

        # swap/negate limits if present
        limit_elem = next((c for c in joint if tagname(c.tag) == 'limit'), None)
        if limit_elem is not None:
            lower = limit_elem.attrib.get('lower')
            upper = limit_elem.attrib.get('upper')
            def to_float(s):
                try: return float(s)
                except Exception: return None
            if lower is not None and upper is not None:
                lo, up = to_float(lower), to_float(upper)
                if lo is not None and up is not None and not dryrun:
                    limit_elem.set('lower', fmt_num(-up))
                    limit_elem.set('upper', fmt_num(-lo))
        print(f"  * {jname}: axis {old_axis} -> {new_axis}")
        changed += 1

if changed == 0:
    print("  * No joints modified (nothing negative on a single axis)")

if not dryrun:
    xml_bytes = ET.tostring(root, encoding='utf-8')
    if not xml_bytes.lstrip().startswith(b'<?xml'):
        xml_bytes = b'<?xml version=\"1.0\"?>\n' + xml_bytes
    if inplace:
        with open(in_path, 'wb') as f: f.write(xml_bytes)
        print(f"✅ Updated in-place: {in_path}")
    else:
        with open(out_path, 'wb') as f: f.write(xml_bytes)
        print(f"✅ Wrote: {out_path}")
PY
done
