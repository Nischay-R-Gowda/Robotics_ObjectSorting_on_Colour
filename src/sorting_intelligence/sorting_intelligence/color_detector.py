"""Lightweight color-blob detection for RGB camera frames."""


def detect_color_blob(
    data, width, height, row_step, encoding, sample_step=2,
    min_samples=6, min_confidence=0.65,
):
    """Return a central-ROI color centroid, or None when no blob is clear.

    The output x/y values are image pixels, not world coordinates. This simple
    detector is suitable for the fixed demonstration camera, not general vision.
    """
    if width <= 0 or height <= 0 or sample_step <= 0:
        return None

    enc = encoding.lower()
    if enc == "rgb8":
        channels, order = 3, (0, 1, 2)
    elif enc == "bgr8":
        channels, order = 3, (2, 1, 0)
    elif enc == "rgba8":
        channels, order = 4, (0, 1, 2)
    elif enc == "bgra8":
        channels, order = 4, (2, 1, 0)
    else:
        raise ValueError(f"unsupported image encoding: {encoding}")

    row_step = row_step or width * channels
    if row_step < width * channels or len(data) < row_step * height:
        return None

    # Overhead workcell view: isolate the red cube at x=0.85 m, y=0 m.
    # The coloured bins lie above/below this narrow pickup strip.
    x0, x1 = int(width * 0.57), int(width * 0.73)
    y0, y1 = int(height * 0.36), int(height * 0.64)
    counts = {"red_object": 0, "green_object": 0, "blue_object": 0}
    sums = {label: [0, 0] for label in counts}

    for y in range(y0, y1, sample_step):
        row = y * row_step
        for x in range(x0, x1, sample_step):
            offset = row + x * channels
            r = int(data[offset + order[0]])
            g = int(data[offset + order[1]])
            b = int(data[offset + order[2]])
            peak = max(r, g, b)
            if peak < 70:
                continue
            if r >= 1.60 * g and r >= 1.35 * b and r - g >= 60:
                label = "red_object"
            elif g >= 1.35 * r and g >= 1.35 * b and g - max(r, b) >= 45:
                label = "green_object"
            elif b >= 1.35 * r and b >= 1.35 * g and b - max(r, g) >= 45:
                label = "blue_object"
            else:
                continue
            counts[label] += 1
            sums[label][0] += x
            sums[label][1] += y

    label = max(counts, key=counts.get)
    total = sum(counts.values())
    count = counts[label]
    if count < min_samples or total == 0:
        return None
    confidence = count / total
    if confidence < min_confidence:
        return None

    return {
        "label": label,
        "x": round(sums[label][0] / count),
        "y": round(sums[label][1] / count),
        "confidence": round(confidence, 3),
        "pixels": count,
        "coordinate_frame": "image_pixels",
    }
