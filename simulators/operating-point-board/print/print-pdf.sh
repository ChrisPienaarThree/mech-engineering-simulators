#!/bin/bash
# Converts every SVG and HTML in this folder to PDF at 100 % scale with headless Chrome (macOS path; adjust for Windows).
cd "$(dirname "$0")"
CH="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
for f in *.svg; do
  base="${f%.svg}"; size=$(sed -nE 's/.*width="([0-9.]+)mm" height="([0-9.]+)mm".*/\1mm \2mm/p' "$f" | head -1)
  printf '<!doctype html><html><head><meta charset="utf-8"><style>@page{size:%s;margin:0}html,body{margin:0}img{display:block;width:100%%;height:100%%}</style></head><body><img src="%s"></body></html>' "$size" "$f" > "_$base.html"
  "$CH" --headless=new --disable-gpu --no-pdf-header-footer --print-to-pdf="$base.pdf" "file://$PWD/_$base.html" 2>/dev/null; rm -f "_$base.html"
done
for f in scenario-cards facilitator-key team-record-A4; do "$CH" --headless=new --disable-gpu --no-pdf-header-footer --print-to-pdf="$f.pdf" "file://$PWD/$f.html" 2>/dev/null; done
ls -la *.pdf
