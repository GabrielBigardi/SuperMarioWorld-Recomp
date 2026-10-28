# Converts every .ppm in the given directory to a 3x nearest-neighbour upscaled .png
import sys, glob, os
from PIL import Image
d = sys.argv[1] if len(sys.argv) > 1 else "out"
for p in sorted(glob.glob(os.path.join(d, "*.ppm"))):
    im = Image.open(p)
    im = im.resize((im.width * 3, im.height * 3), Image.NEAREST)
    im.save(p[:-4] + ".png")
    print("png:", p[:-4] + ".png")
