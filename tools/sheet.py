"""Turn raw 256x224 XRGB frame dumps into a PNG contact sheet: sheet.py out.png f1.raw f2.raw ..."""
import sys
import numpy as np
from PIL import Image
out, files = sys.argv[1], sys.argv[2:]
imgs = []
for f in files:
    a = np.fromfile(f, dtype=np.uint32).reshape(224, 256)
    rgb = np.stack([(a >> 16) & 255, (a >> 8) & 255, a & 255], -1).astype(np.uint8)
    imgs.append(Image.fromarray(rgb))
W = min(5, len(imgs))
H = (len(imgs) + W - 1) // W
sheet = Image.new('RGB', (256 * W, 224 * H))
for i, im in enumerate(imgs):
    sheet.paste(im, ((i % W) * 256, (i // W) * 224))
sheet.save(out)
print(out, len(imgs))
