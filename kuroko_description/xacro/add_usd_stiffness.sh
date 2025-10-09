#!/usr/bin/env bash
# add_sdf_stiffness.sh
# Description:
#   In-place edit of an SDF file:
#     - For each <limit> block followed by its <dynamics> block,
#       if BOTH effort != 0 and velocity != 0:
#         * Insert <stiffness> = effort * 8.0 (treated as default P gain) right after <velocity>.
#         * Replace <damping> with effort / velocity.
#       Otherwise: leave the entire block untouched.
#   Original indentation is preserved.
#
# Usage:
#   ./add_sdf_stiffness.sh path/to/model.sdf
#
# Notes:
#   - Expects the following order inside the target blocks:
#       <limit>:   <lower>, <upper>, <effort>, <velocity>, </limit>
#       <dynamics>: <damping>, <friction>, <spring_reference>, <spring_stiffness>, </dynamics>
#   - To keep a backup, change Perl option `-i` to `-i.bak` (creates *.bak files).
#   - Numeric precision is not enforced (Perl default float formatting).

set -euo pipefail

if [[ $# -ne 1 || ! -f "$1" ]]; then
  echo "Usage: $0 <file.sdf>" >&2
  exit 1
fi

file="$1"

perl -0777 -i -pe '
  # Match a <limit> block immediately followed by its <dynamics> block.
  s{
    ([ \t]*)<limit>\s*\n                              # (1)  indent of <limit>
    ([ \t]*)<lower>(-?[\d.]+)</lower>\s*\n            # (2)(3) indent+value lower
    ([ \t]*)<upper>(-?[\d.]+)</upper>\s*\n            # (4)(5) indent+value upper
    ([ \t]*)<effort>(-?[\d.]+)</effort>\s*\n          # (6)(7) indent+value effort
    ([ \t]*)<velocity>(-?[\d.]+)</velocity>\s*\n      # (8)(9) indent+value velocity
    ([ \t]*)</limit>\s*\n                             # (10)   indent of </limit>
    ([ \t]*)<dynamics>\s*\n                           # (11)   indent of <dynamics>
    ([ \t]*)<damping>(-?[\d.]+)</damping>\s*\n        # (12)(13) indent+old damping
    ([ \t]*)<friction>(-?[\d.]+)</friction>\s*\n      # (14)(15) indent+friction
    ([ \t]*)<spring_reference>(-?[\d.]+)</spring_reference>\s*\n  # (16)(17)
    ([ \t]*)<spring_stiffness>(-?[\d.]+)</spring_stiffness>\s*\n  # (18)(19)
    ([ \t]*)</dynamics>                               # (20)   indent of </dynamics>
  }{
    # Unpack captures for readability
    my ($iL,
        $ilow,$low,
        $iup,$up,
        $ie,$eff,
        $iv,$vel,
        $iLend,
        $iDyn,
        $iDamp,$damp_old,
        $iFric,$fric,
        $iSR,$sr,
        $iSS,$ss,
        $iDynEnd) =
      ($1,$2,$3,$4,$5,$6,$7,$8,$9,$10,$11,$12,$13,$14,$15,$16,$17,$18,$19,$20);

    # If either effort or velocity is zero, do nothing (keep block unchanged).
    if (($eff+0) == 0 || ($vel+0) == 0) {
      return $&;  # entire original match
    }

    # Compute:
    #   stiffness := effort * 8.0  (treated as default P gain)
    #   damping   := effort / velocity
    my $stiff = $eff * 8.0;
    my $damp  = $eff / $vel;

    # Rebuild blocks, preserving original indentation. No extra comments are injected.
    join("",
      $iL   . "<limit>\n",
      $ilow . "<lower>$low</lower>\n",
      $iup  . "<upper>$up</upper>\n",
      $ie   . "<effort>$eff</effort>\n",
      $iv   . "<velocity>$vel</velocity>\n",
      $iv   . "<stiffness>$stiff</stiffness>\n",     # inserted default P gain
      $iLend. "</limit>\n",
      $iDyn . "<dynamics>\n",
      $iDamp. "<damping>$damp</damping>\n",
      $iFric. "<friction>$fric</friction>\n",
      $iSR  . "<spring_reference>$sr</spring_reference>\n",
      $iSS  . "<spring_stiffness>$ss</spring_stiffness>\n",
      $iDynEnd . "</dynamics>"
    );
  }egx;
' "$file"

echo "Updated: $file"
