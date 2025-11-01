#!/bin/bash
# Usage: ./set_usd_unit_meter.sh <path-to-usda>
# Safely set metersPerUnit=1.0 in the USDA header block.

set -euo pipefail

if [ $# -ne 1 ]; then
  echo "Usage: $0 <usda file>"
  exit 1
fi

FILE="$1"
if [ ! -f "$FILE" ]; then
  echo "Error: file not found: $FILE" >&2
  exit 1
fi

# 1) USDA判定
first_line="$(head -n1 "$FILE" || true)"
if ! echo "$first_line" | grep -q '^#usda'; then
  echo "Error: not a USDA text file (missing #usda header): $FILE" >&2
  echo "Hint: usdcat <.usd> > <.usda> でテキスト化してください。" >&2
  exit 1
fi

# 2) ヘッダ範囲の抽出（最初の'('から対応する')'まで）
#    その範囲に metersPerUnit があるか/ないかを判定して処理
tmp="$(mktemp)"; trap 'rm -f "$tmp"' EXIT

awk -v OFS="" '
  BEGIN{in_hdr=0; depth=0}
  NR==1 {print; next}
  {
    line=$0
    if (in_hdr==0) {
      if (match(line,/^\(/)) { in_hdr=1; depth=1 }
      print line > "'$tmp'.after"  # とりあえず全部 after に
      next
    } else if (in_hdr==1) {
      # ヘッダ行は別ファイルに貯める
      print line >> "'$tmp'.hdr"
      # 括弧深さトラッキング（単純な行頭/行末想定, 一般的USDAでOK）
      if (match(line,/^\(/)) depth++
      if (match(line,/^\)/)) depth--
      if (depth==0) in_hdr=2
      next
    } else {
      print line >> "'$tmp'.after"
    }
  }
' "$FILE"

# ヘッダが無ければ空のヘッダを作る（稀ケース）
if [ ! -s "$tmp.hdr" ]; then
  printf "(\n)\n" > "$tmp.hdr"
fi

# 3) ヘッダに metersPerUnit があれば値置換、なければ閉じ括弧の直前へ挿入
if grep -q 'metersPerUnit' "$tmp.hdr"; then
  # 行内の数値を 1.0 に正規化（余分な空白も吸収）
  sed -E -i 's/(metersPerUnit[[:space:]]*=[[:space:]]*)[0-9.]+/\11.0/' "$tmp.hdr"
else
  awk '
    BEGIN{inserted=0}
    /^\)/ && inserted==0 { print "    metersPerUnit = 1.0"; inserted=1 }
    { print }
    END{
      if (inserted==0) {
        # 念のため最後に追記（理論上ここは来ない）
        print "    metersPerUnit = 1.0"
      }
    }
  ' "$tmp.hdr" > "$tmp.hdr.new" && mv "$tmp.hdr.new" "$tmp.hdr"
fi

# 4) 再結合（1行目 + ヘッダ + 以降）
{
  head -n1 "$FILE"
  cat "$tmp.hdr"
  cat "$tmp.after"
} > "$FILE.tmp"

mv "$FILE.tmp" "$FILE"
echo "✅ metersPerUnit set to 1.0 in $FILE"
