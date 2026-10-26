#!/usr/bin/env bash
set -euo pipefail

colour="${1:-}"
case "$colour" in
  red|blue|green) ;;
  *) echo "Usage: $0 red|blue|green" >&2; exit 2 ;;
esac
world="$HOME/robotics_ws/src/robot_description/worlds/sorting_world.sdf"
if [[ ! -f "$world" ]]; then
  echo "World file not found: $world" >&2
  exit 1
fi
python3 - "$world" "$colour" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
colour = sys.argv[2]
text = path.read_text()
pattern = re.compile(r'<model name="(?:red|blue|green)_object">.*?</model>', re.S)
matches = list(pattern.finditer(text))
if len(matches) != 1:
    raise SystemExit(f"Expected one demo object, found {len(matches)}")

rgb = {"red": "1 0 0 1", "blue": "0 0 1 1", "green": "0 1 0 1"}[colour]
model = matches[0].group(0)
model = re.sub(r'<model name="(?:red|blue|green)_object">', f'<model name="{colour}_object">', model, count=1)
model, ambient_count = re.subn(r'<ambient>(?:1 0 0 1|0 0 1 1|0 1 0 1)</ambient>', f'<ambient>{rgb}</ambient>', model, count=1)
model, diffuse_count = re.subn(r'<diffuse>(?:1 0 0 1|0 0 1 1|0 1 0 1)</diffuse>', f'<diffuse>{rgb}</diffuse>', model, count=1)
if ambient_count != 1 or diffuse_count != 1:
    raise SystemExit("Could not identify the demo cube's ambient/diffuse material")
text = text[:matches[0].start()] + model + text[matches[0].end():]
path.write_text(text)
print(f"Demo cube set to {colour}; pose and size were kept unchanged.")
PY