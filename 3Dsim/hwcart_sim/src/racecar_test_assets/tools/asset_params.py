"""Single source of truth for racecar_test_assets geometry/material parameters.

Dimensions the rules specify are marked RULE; everything the rules do not
define is an ASSUMPTION and is also listed in docs/assumptions.md.
check_models.py cross-checks the shipped SDF/STL files against these numbers,
and generate_cone_mesh.py regenerates the STL from them, so a change here
must be followed by regenerating the meshes and updating the model.sdf files.
"""
import math

# ---- competition cone (RULE) ----
CONE_TOTAL_HEIGHT = 0.63           # m, rule: total height including base
CONE_BASE_SIZE = 0.28              # m, rule: square base outer envelope (0.28 x 0.28)
# rule: no white retro-reflective band is added to the model

# ---- competition cone (ASSUMPTIONS) ----
CONE_BASE_THICKNESS = 0.05         # m, assumption
CONE_FRUSTUM_BOTTOM_DIAM = 0.24    # m, assumption; must be <= CONE_BASE_SIZE
CONE_FRUSTUM_TOP_DIAM = 0.08       # m, assumption
CONE_SEGMENTS = 24                 # polygonal approximation of the circular frustum

# ---- blue board ----
BOARD_LENGTH = 0.80                # m, rule
BOARD_HEIGHT = 0.50                # m, rule
BOARD_THICKNESS = 0.02             # m, assumption (rule unconfirmed)

# ---- grey distance reference box (test fixture, not a race prop) ----
REFBOX_SIZE = (0.5, 1.0, 1.0)      # m (x, y, z)

# ---- traffic light (ASSUMPTIONS: all dimensions) ----
# Pole + housing + three spherical lenses (red/yellow/green, top-to-bottom).
# STATIC model: the three lenses are always-on coloured geometry; there is NO
# runtime switch interface (documented as such, not merely untested).
# Origin: centre of the pole bottom face (pose z=0 -> stands on the ground).
# Local axes: housing front faces +Y (yaw rotates it in the ground plane).
TL_POLE_RADIUS = 0.05              # m
TL_POLE_HEIGHT = 1.80              # m
TL_HOUSING_SIZE = (0.30, 0.16, 0.66)   # m (x=width, y=thickness, z=height)
TL_HOUSING_BOTTOM_Z = 1.80         # m (housing sits on top of the pole)
TL_LENS_RADIUS = 0.07              # m (spherical lens, always-on colour)
TL_LENS_DZ = 0.20                  # m (vertical spacing between lens centres)
TL_LENS_Y = 0.095                  # m (lens centre offset along +Y, on the front)

# ---- A/B signs (ASSUMPTIONS: all dimensions) ----
# Pole + rectangular face with a self-generated letter texture (white
# background, black "A"/"B"; tools/generate_sign_texture.py, no external
# asset). Origin: centre of the pole bottom face. Face centre z above ground.
SIGN_POLE_RADIUS = 0.04            # m
SIGN_POLE_HEIGHT = 1.40            # m
SIGN_FACE_SIZE = (0.50, 0.03, 0.70)    # m (x=width, y=thickness, z=height)
SIGN_FACE_CENTER_Z = 1.60          # m

# ---- lane line (ASSUMPTIONS: all dimensions) ----
# Thin white box laid flat on the ground. Origin: centre of the bottom face
# (pose z=0 -> bottom face on the ground). Local axes: X = length, Y = width.
LANE_LINE_LENGTH = 3.00            # m (X)
LANE_LINE_WIDTH = 0.15             # m (Y)
LANE_LINE_THICKNESS = 0.01         # m (Z)

# ---- materials (ASSUMPTIONS: vivid, camera-distinguishable) ----
MATERIALS = {
    'cone_red':      {'ambient': '0.60 0.05 0.05 1', 'diffuse': '0.85 0.10 0.10 1'},
    'cone_blue':     {'ambient': '0.03 0.10 0.60 1', 'diffuse': '0.05 0.15 0.85 1'},
    'blue_board':    {'ambient': '0.03 0.10 0.60 1', 'diffuse': '0.05 0.15 0.85 1'},
    'reference_box': {'ambient': '0.40 0.40 0.40 1', 'diffuse': '0.60 0.60 0.60 1'},
    # traffic light (multi-material model; keys are <model>/<part>)
    'traffic_light/pole':    {'ambient': '0.20 0.20 0.20 1', 'diffuse': '0.35 0.35 0.35 1'},
    'traffic_light/housing': {'ambient': '0.15 0.15 0.15 1', 'diffuse': '0.25 0.25 0.25 1'},
    'traffic_light/red':     {'ambient': '0.60 0.05 0.05 1', 'diffuse': '0.85 0.10 0.10 1'},
    'traffic_light/yellow':  {'ambient': '0.60 0.55 0.05 1', 'diffuse': '0.85 0.80 0.10 1'},
    'traffic_light/green':   {'ambient': '0.05 0.50 0.05 1', 'diffuse': '0.10 0.75 0.15 1'},
    # signs (face is textured white; pole grey)
    'sign/pole':  {'ambient': '0.20 0.20 0.20 1', 'diffuse': '0.35 0.35 0.35 1'},
    'sign/face':  {'ambient': '0.70 0.70 0.70 1', 'diffuse': '0.90 0.90 0.90 1'},
    # lane line (white)
    'lane_line':  {'ambient': '0.60 0.60 0.60 1', 'diffuse': '0.90 0.90 0.90 1'},
}
SPECULAR = '0.10 0.10 0.10 1'

# ---- derived cone geometry ----
CONE_FRUSTUM_HEIGHT = CONE_TOTAL_HEIGHT - CONE_BASE_THICKNESS


def cone_cross_section(z):
    """Collision cross-section of the cone at height z (metres above ground).

    The frustum is a CONE_SEGMENTS-gon whose circumradius interpolates
    linearly, so the 'circle' numbers are approximations within
    1 - cos(pi/24) = 0.86 % (polygon inradius vs circumradius).
    """
    if z < -1e-12 or z > CONE_TOTAL_HEIGHT + 1e-12:
        return {'z': z, 'part': 'none', 'note': 'above/below the cone'}
    if z <= CONE_BASE_THICKNESS:
        return {'z': z, 'part': 'square_base',
                'size': CONE_BASE_SIZE,
                'area': CONE_BASE_SIZE ** 2}
    r0 = CONE_FRUSTUM_BOTTOM_DIAM / 2.0
    r1 = CONE_FRUSTUM_TOP_DIAM / 2.0
    t = (z - CONE_BASE_THICKNESS) / CONE_FRUSTUM_HEIGHT
    r = r0 + (r1 - r0) * t
    return {'z': z, 'part': 'frustum',
            'circumradius': r,
            'diameter': 2 * r,
            'area': math.pi * r * r,
            'polygon_inradius': r * math.cos(math.pi / CONE_SEGMENTS)}
