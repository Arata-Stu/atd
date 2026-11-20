"""Track-facing, unlit marshal light for the Monza lap, in RC metres."""

from monza_trackside import _beam, _post

PANEL_FRAME_COLOR = (.37, .38, .37)
PANEL_FACE_COLOR = (.062, .065, .068)


def panel(center, forward):
    """Return support/casing faces and dark display faces for one installed panel."""
    x, y = center
    dx, dy = forward

    def at(lateral, ahead):
        return x - dy * lateral + dx * ahead, y + dx * lateral + dy * ahead

    frame = []
    _post(frame, x, y, dx, dy, .012, .014, 0., .265)
    frame.extend(_beam(at(-.052, 0), at(.052, 0), .018, .25, .332))
    frame.extend(_beam(at(-.054, -.006), at(.054, -.006), .029, .332, .336))
    # The graphic area follows the proportions of FIA 3504-2019, §4.7.
    # The housing and mounting height are approximate. Leave the display off.
    corners = ((-.047, .254), (-.047, .328), (.047, .328), (.047, .254))
    face = [(*at(lateral, -.0095), z) for lateral, z in corners]
    return frame, [face]
