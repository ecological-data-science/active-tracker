COLLAR_MAJOR_RADIUS = 350
COLLAR_MINOR_RADIUS = 140
COLLAR_WIDTH = 50

CAVITY_HEIGHT = 5

# this is the height of the inner edge that fits inside the lid
INNER_EDGE_HEIGHT = 3
# this is the height of the outer edge just below the lid
OUTER_EDGE_HEIGHT = 2

# this is the thickness of the curved base
PLATE_THICKNESS = 10

# this is the thickness of the base once the inset has been cut out
FLOOR_THICKNESS = 3

# this is the location of the top of the base
# the curved plate is 10mm thick and its top is at 0
# the floor is 3mm thick
MAX_BASE_Y = -PLATE_THICKNESS + FLOOR_THICKNESS + CAVITY_HEIGHT

CHIP_WIDTH = 88
CHIP_LENGTH = 48
CHIP_HEIGHT = 18

CHIP_TOLERANCE = 0.1
PCB_HEIGHT = 2

BASE_TOP_WIDTH = CHIP_WIDTH + 13
BASE_TOP_LENGTH = CHIP_LENGTH + 13

GROOVE_TOLERANCE = 0.5
GROOVE_HEIGHT = 3.0

SEAL_LENGTH = CHIP_LENGTH + 5
SEAL_WIDTH = CHIP_WIDTH + 5
SEAL_HEIGHT = 2

SIDE_OFFSET = 27.5
FRONT_OFFSET = -4.0
HEX_NUT_DIAM = 9.0
BIG_HEX_NUT_DIAM = 13.0
SCREW_ANGLE = 43


SCREW_LOCATION_W = 83  # PANEL_WIDTH + 13 # 24 +GROW
SCREW_LOCATION_L = 63  # PANEL_LENGTH + 8

PANEL_WIDTH = 65
PANEL_LENGTH = 65
PANEL_HEIGHT = 4


LID_TOP_LENGTH = PANEL_LENGTH + 6
LID_TOP_WIDTH = PANEL_WIDTH + 20

LID_BOTTOM_LENGTH = BASE_TOP_LENGTH + 2.5
LID_BOTTOM_WIDTH = BASE_TOP_WIDTH + 8

ROOF_THICKNESS = 2
# this is the location of the highest point of the lid
MAX_LID_Y = (
    PANEL_HEIGHT
    + ROOF_THICKNESS
    + CHIP_HEIGHT
    + FLOOR_THICKNESS
    - PLATE_THICKNESS
)
