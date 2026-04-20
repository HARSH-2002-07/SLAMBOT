"""Week 9 preflight: round-trip an image through Gemini and print structured output.

This is deliberately NOT a ROS2 node. It proves:
  1. GEMINI_API_KEY works from this machine
  2. gemini-2.5-flash returns parseable JSON for a "find this object" prompt

Gemini multimodal 2.5 returns bounding boxes in its native format:
  [y_min, x_min, y_max, x_max] normalized to 0–1000 (not pixels, not xyxy).
This script requests that format explicitly and converts to pixel xyxy for
downstream use.

Usage:
    export GEMINI_API_KEY=AI...
    python3 tools/gemini_probe.py <image_path> "<target description>" [--draw out.png]

Example:
    python3 tools/gemini_probe.py /tmp/gazebo_red.png "the red block"
    python3 tools/gemini_probe.py /tmp/gazebo_red.png "the red block" --draw /tmp/gazebo_red_bbox.png

Install deps:
    pip install --user google-genai pillow
"""

import argparse
import json
import os
import sys
import time

import PIL.Image
import PIL.ImageDraw
from google import genai
from google.genai import errors as genai_errors
from google.genai import types


MODEL_NAME = "gemini-2.5-flash"
RETRY_STATUSES = (429, 503)  # rate-limit and capacity errors
MAX_ATTEMPTS = 3

PROMPT_TEMPLATE = (
    "Detect the 2D bounding box of the single best match for this target "
    "in the image: {target!r}. "
    "Return exactly one object with fields: "
    "label (str), box_2d ([y_min, x_min, y_max, x_max] normalized 0-1000), "
    "confidence (0.0-1.0), notes (short str). "
    "If the target is not visible, set confidence to 0.0 and box_2d to "
    "[0, 0, 0, 0]."
)

# Server-enforced JSON schema — stops the model from returning 5-element
# arrays or other shape drift we saw with prompt-only constraints.
RESPONSE_SCHEMA = {
    "type": "OBJECT",
    "properties": {
        "label": {"type": "STRING"},
        "box_2d": {
            "type": "ARRAY",
            "items": {"type": "INTEGER"},
            "minItems": 4,
            "maxItems": 4,
        },
        "confidence": {"type": "NUMBER"},
        "notes": {"type": "STRING"},
    },
    "required": ["label", "box_2d", "confidence"],
}


def denormalize(box_2d, img_w: int, img_h: int) -> list[int]:
    """[y_min, x_min, y_max, x_max] in 0–1000 → [x1, y1, x2, y2] in pixels."""
    if isinstance(box_2d, list) and len(box_2d) == 1 and isinstance(box_2d[0], list):
        box_2d = box_2d[0]  # unwrap accidental nesting
    if not (isinstance(box_2d, list) and len(box_2d) == 4):
        raise ValueError(f"box_2d has unexpected shape: {box_2d!r}")
    y1, x1, y2, x2 = box_2d
    return [
        round(x1 / 1000 * img_w),
        round(y1 / 1000 * img_h),
        round(x2 / 1000 * img_w),
        round(y2 / 1000 * img_h),
    ]


def draw_bbox(img: PIL.Image.Image, xyxy: list[int], label: str,
              out_path: str) -> None:
    annotated = img.copy()
    draw = PIL.ImageDraw.Draw(annotated)
    draw.rectangle(xyxy, outline="red", width=4)
    draw.text((xyxy[0] + 4, max(0, xyxy[1] - 14)), label, fill="red")
    annotated.save(out_path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("image_path")
    ap.add_argument("target")
    ap.add_argument("--draw", metavar="OUT_PNG", default=None,
                    help="Write annotated image with bbox overlay")
    args = ap.parse_args()

    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        print("ERROR: set GEMINI_API_KEY in the environment.", file=sys.stderr)
        return 2
    if not os.path.isfile(args.image_path):
        print(f"ERROR: image not found: {args.image_path}", file=sys.stderr)
        return 2

    img = PIL.Image.open(args.image_path).convert("RGB")
    prompt = PROMPT_TEMPLATE.format(target=args.target)

    client = genai.Client(api_key=api_key)
    t0 = time.time()
    response = None
    for attempt in range(1, MAX_ATTEMPTS + 1):
        try:
            response = client.models.generate_content(
                model=MODEL_NAME,
                contents=[img, prompt],
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=RESPONSE_SCHEMA,
                ),
            )
            break
        except genai_errors.APIError as e:
            if e.code in RETRY_STATUSES and attempt < MAX_ATTEMPTS:
                backoff = 2 ** attempt
                print(f"WARN: {e.code} from Gemini (attempt {attempt}/"
                      f"{MAX_ATTEMPTS}); retrying in {backoff}s…",
                      file=sys.stderr)
                time.sleep(backoff)
                continue
            raise
    elapsed = time.time() - t0

    raw = response.text
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as e:
        print(f"FAIL: model returned non-JSON after {elapsed:.2f}s:\n{raw}",
              file=sys.stderr)
        print(f"JSONDecodeError: {e}", file=sys.stderr)
        return 1

    required = {"label", "box_2d", "confidence"}
    missing = required - set(parsed.keys())
    if missing:
        print(f"FAIL: missing fields in response: {missing}", file=sys.stderr)
        print(json.dumps(parsed, indent=2), file=sys.stderr)
        return 1

    box_2d = parsed["box_2d"]
    print(f"latency_s       : {elapsed:.2f}")
    print(f"image           : {args.image_path} ({img.width}x{img.height})")
    print(f"target          : {args.target}")
    print(f"model           : {MODEL_NAME}")
    print(f"label           : {parsed['label']}")
    print(f"confidence      : {parsed['confidence']:.2f}")
    print(f"box_2d (raw)    : {box_2d}")
    print(f"notes           : {parsed.get('notes', '')}")

    xyxy = denormalize(box_2d, img.width, img.height)
    print(f"bbox_xyxy (px)  : {xyxy}")

    if args.draw:
        draw_bbox(img, xyxy, parsed["label"], args.draw)
        print(f"annotated       : {args.draw}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
