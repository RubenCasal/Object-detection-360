import numpy as np

class BBoxSmoother:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.prev_box = None

    def update(self, new_box):
        if self.prev_box is None:
            self.prev_box = new_box
        else:
            self.prev_box = [
                self.alpha * n + (1 - self.alpha) * p
                for n, p in zip(new_box, self.prev_box)
            ]
        return [int(v) for v in self.prev_box]

# initialize this once outside the loop
smoother = BBoxSmoother(alpha=0.3)

def crop_and_wrap(frame, bbox, pad_ratio=0.6):
    h_img, w_img = frame.shape[:2]

    # smooth bbox over time
    bbox = smoother.update(bbox)
    x, y, w, h = bbox

    # compute padded centered crop
    pad_w = int(w * pad_ratio)
    pad_h = int(h * pad_ratio)

    cx, cy = x + w // 2, y + h // 2
    crop_w = w + 2 * pad_w
    crop_h = h + 2 * pad_h

    x1 = (cx - crop_w // 2) % w_img
    x2 = (cx + crop_w // 2) % w_img
    y1 = max(cy - crop_h // 2, 0)
    y2 = min(cy + crop_h // 2, h_img)

    if x1 > x2:  # wrap horizontally
        left = frame[y1:y2, x1:]
        right = frame[y1:y2, :x2]
        cropped = np.hstack([left, right])
        x1_draw, x2_draw = 0, cropped.shape[1]
    else:
        cropped = frame[y1:y2, x1:x2]
        x1_draw, x2_draw = x1, x2

    return cropped, (x1_draw, y1, x2_draw, y2)
