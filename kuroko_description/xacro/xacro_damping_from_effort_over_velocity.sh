#!/usr/bin/env bash
# Update (or create) <dynamics damping="..."> for joints in xacro/URDF files,
# setting damping = effort / velocity from the joint's <limit> element.
# - Preserves XML namespaces (keeps 'xacro:' prefix).
# - Supports multiple files via wildcards (e.g., *.xacro).
# - In-place (-i) and dry-run (-n), verbose (-v) modes.
# Usage:
#   ./xacro_damping_from_effort_over_velocity.sh -n -v file1.xacro ...
#   ./xacro_damping_from_effort_over_velocity.sh -i *.xacro
#   ./xacro_damping_from_effort_over_velocity.sh file1.xacro  # writes *_damped.xacro

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
  if [[ ! -f "$IN" ]]; then
    echo "Skipping: $IN (not found)" >&2
    continue
  fi

  OUT=""
  if [[ $INPLACE -eq 0 ]]; then
    base="${IN%.*}"
    OUT="${base}_damped.xacro"
  fi

  echo "==> Processing: $IN"
  python3 - "$IN" "${OUT}" "${INPLACE}" "${DRYRUN}" "${VERBOSE}" <<'PY'
import sys, math
from xml.etree import ElementTree as ET

in_path  = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] else None
inplace  = bool(int(sys.argv[3])) if len(sys.argv) > 3 else False
dryrun   = bool(int(sys.argv[4])) if len(sys.argv) > 4 else False
verbose  = bool(int(sys.argv[5])) if len(sys.argv) > 5 else False

def tagname(t): 
    # Get local name without namespace
    return t.split('}')[-1]

def to_float(s):
    try:
        return float(s)
    except Exception:
        return None

def fmt_num(x):
    # Compact float formatting (no trailing zeros)
    if math.isfinite(x) and abs(x - round(x)) < 1e-12:
        return str(int(round(x)))
    return f"{x:.6f}".rstrip('0').rstrip('.')

# Preserve original prefixes so we don't get ns0:
nsmap = {}
it = ET.iterparse(in_path, events=('start','start-ns'))
for ev, obj in it:
    if ev == 'start-ns':
        prefix, uri = obj
        nsmap[prefix or ''] = uri
root = it.root
for prefix, uri in nsmap.items():
    ET.register_namespace(prefix, uri)

updated, skipped = 0, 0

for joint in root.iter():
    if tagname(joint.tag) != 'joint':
        continue
    jname = joint.attrib.get('name', '(unnamed)')

    # Find <limit> with effort and velocity
    limit_elem = next((c for c in joint if tagname(c.tag) == 'limit'), None)
    if limit_elem is None:
        if verbose: print(f"  - {jname}: no <limit>", file=sys.stderr)
        skipped += 1
        continue

    effort_s  = limit_elem.attrib.get('effort')
    vel_s     = limit_elem.attrib.get('velocity')
    e = to_float(effort_s) if effort_s is not None else None
    v = to_float(vel_s)    if vel_s is not None    else None

    if e is None or v is None or v == 0:
        if verbose:
            why = "missing" if (effort_s is None or vel_s is None) else ("non-numeric" if (e is None or v is None) else "velocity==0")
            print(f"  - {jname}: cannot compute damping (effort={effort_s!r}, velocity={vel_s!r}) [{why}]", file=sys.stderr)
        skipped += 1
        continue

    damping = e / v

    # Find or create <dynamics>
    dyn_elem = next((c for c in joint if tagname(c.tag) == 'dynamics'), None)
    pre = None
    old = None
    if dyn_elem is None:
        if verbose: print(f"  . {jname}: creating <dynamics>", file=sys.stderr)
        dyn_elem = ET.Element('dynamics')
        # Try to insert dynamics near limit for readability
        try:
            idx = list(joint).index(limit_elem)
            joint.insert(idx + 1, dyn_elem)
        except Exception:
            joint.append(dyn_elem)
        pre = "(new)"
    else:
        pre = "(exists)"
        old = dyn_elem.attrib.get('damping')

    # Set damping (preserve friction if present)
    if not dryrun:
        dyn_elem.set('damping', fmt_num(damping))

    old_txt = "none" if old is None else old
    print(f"  * {jname}: damping {pre} {old_txt} -> {fmt_num(damping)}  (effort={effort_s}, velocity={vel_s})")
    updated += 1

# Summary and write-out
if updated == 0:
    print("  * No joints updated (no computable effort/velocity found)")
if not dryrun:
    xml_bytes = ET.tostring(root, encoding='utf-8')
    if not xml_bytes.lstrip().startswith(b'<?xml'):
        xml_bytes = b'<?xml version="1.0"?>\n' + xml_bytes
    if inplace:
        with open(in_path, 'wb') as f: f.write(xml_bytes)
        print(f"✅ Updated in-place: {in_path}")
    else:
        with open(out_path, 'wb') as f: f.write(xml_bytes)
        print(f"✅ Wrote: {out_path}")
PY
done
