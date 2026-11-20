"""Bake the Monza lap pavement once from the bundled CC0 asphalt photograph."""
from pathlib import Path

import numpy as np
from PIL import Image


MATERIALS = Path(__file__).resolve().parent / "materials"
SIZE = 2048


def pavement_tone(photo, cells):
    """Expand photographic variations across the tile with continuous edges."""
    samples = np.asarray(photo.resize((cells, cells), Image.Resampling.BOX))
    padded = Image.fromarray(np.pad(samples, 2, mode="wrap"))
    step = SIZE // cells
    width = SIZE + 4 * step
    enlarged = padded.resize((width, width), Image.Resampling.BICUBIC)
    values = np.asarray(enlarged.crop((2*step, 2*step, width-2*step, width-2*step)))
    return (values - values.mean()) / values.std()


def main():
    with Image.open(MATERIALS / "floor.jpg") as source:
        photo = source.convert("L").convert("F")
    # Four 0.8 m aggregate tiles retain the size of the original grains.
    # LANCZOS preserves their shape when baking a 512 px patch from the 1K photo.
    grain = np.asarray(photo.resize((SIZE // 4, SIZE // 4), Image.Resampling.LANCZOS))
    grain = np.tile(grain - grain.mean(), (4, 4))
    # Preserve asphalt grain up close. Weak, larger-scale changes in the
    # binder colour remain visible when the renderer filters out fine grain.
    tone = pavement_tone(photo, 64)
    broad_tone = pavement_tone(photo.transpose(Image.Transpose.ROTATE_90), 16)
    albedo = 117 + 1.7 * grain + 5.5 * tone + 3.5 * broad_tone
    pixels = np.clip(albedo.round(), 0, 255).astype(np.uint8)
    Image.fromarray(pixels).convert("RGB").save(MATERIALS / "monza_asphalt.jpg", quality=91)


if __name__ == "__main__":
    main()
