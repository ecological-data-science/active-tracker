import math
import sys
from math import cos, radians, sin

import BOPTools.JoinFeatures
import FreeCAD as App
import Mesh
import Part
import PartGui
from FreeCAD import Base

from dimensions import *


def build_lid_top(doc):
    ymin = MAX_BASE_Y - INNER_EDGE_HEIGHT

    arduinobox1 = doc.addObject('Part::Wedge', 'arduinobox1')
    arduinobox1.Zmin = -LID_BOTTOM_WIDTH / 2
    arduinobox1.Xmin = -LID_BOTTOM_LENGTH / 2
    arduinobox1.Z2min = -LID_TOP_WIDTH / 2
    arduinobox1.X2min = -LID_TOP_LENGTH / 2
    arduinobox1.Zmax = LID_BOTTOM_WIDTH / 2
    arduinobox1.Xmax = LID_BOTTOM_LENGTH / 2
    arduinobox1.Z2max = LID_TOP_WIDTH / 2
    arduinobox1.X2max = LID_TOP_LENGTH / 2

    arduinobox1.Ymin = ymin
    arduinobox1.Ymax = MAX_LID_Y

    arduinobox1.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = arduinobox1
    __fillets__ = []
    __fillets__.append((2, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((4, 2.00, 2.00))
    __fillets__.append((6, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    __fillets__.append((8, 2.00, 2.00))
    __fillets__.append((11, 2.00, 2.00))
    __fillets__.append((12, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    arduinobox1a = App.ActiveDocument.addObject(
        'Part::Feature', 'arduinobox1a'
    )
    arduinobox1a.Label = 'arduinobox1a'
    arduinobox1a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject('Fillet')
    doc.removeObject('arduinobox1')

    return arduinobox1a


def build_lid_base(doc):
    ymin = -90  # at this point the wedge must be wb x lb
    ymax = MAX_BASE_Y - INNER_EDGE_HEIGHT

    lower_width = 17.1  # 22.5 #25.5
    lower_length = 140 + 10 + 6 + 6.7

    arduinobox2 = doc.addObject('Part::Wedge', 'arduinobox2')
    arduinobox2.Zmin = -lower_length / 2
    arduinobox2.Xmin = -lower_width / 2
    arduinobox2.Z2min = -LID_BOTTOM_WIDTH / 2
    arduinobox2.X2min = -LID_BOTTOM_LENGTH / 2
    arduinobox2.Zmax = lower_length / 2
    arduinobox2.Xmax = lower_width / 2
    arduinobox2.Z2max = LID_BOTTOM_WIDTH / 2
    arduinobox2.X2max = LID_BOTTOM_LENGTH / 2

    arduinobox2.Ymin = ymin
    arduinobox2.Ymax = ymax + 0.01

    arduinobox2.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    CUT_BOX_HEIGHT2 = 100
    lipobox2 = doc.addObject('Part::Box', 'lipobox2')
    LIPO_BOX_HEIGHT2 = 2
    LIPO_BOX_HEIGHT = 3
    lipobox2.Length = BASE_TOP_LENGTH + 4
    lipobox2.Width = BASE_TOP_WIDTH + 0.5
    lipobox2.Height = CUT_BOX_HEIGHT2
    lipobox2.Placement = App.Placement(
        App.Vector(
            -(BASE_TOP_LENGTH + 4) / 2,
            -(BASE_TOP_WIDTH + 0.5) / 2,
            MAX_BASE_Y - INNER_EDGE_HEIGHT - CUT_BOX_HEIGHT2,
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = lipobox2
    __fillets__ = []
    __fillets__.append((10, 1.50, 1.50))
    __fillets__.append((12, 1.50, 1.50))

    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    lipobox3 = App.ActiveDocument.addObject('Part::Feature', 'lipobox3')
    lipobox3.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject('Fillet')
    doc.removeObject('lipobox2')

    doc.recompute()

    cutchipbox2 = doc.addObject('Part::Cut', 'cutchipbox2')
    cutchipbox2.Base = arduinobox2
    cutchipbox2.Tool = lipobox3
    doc.recompute()

    arduinobox3 = doc.addObject('Part::Feature', 'arduinobox3')
    arduinobox3.Label = 'arduinobox3'
    arduinobox3.Shape = Part.Solid(Part.Shell(cutchipbox2.Shape.Faces))

    doc.removeObject('lipobox3')
    doc.removeObject('cutchipbox2')
    doc.removeObject('arduinobox2')

    # MAKE THE OUTER ELLIPSE FOR COLLAR
    outer_ellipse = doc.addObject('Part::Ellipse', 'outer_ellipse')
    outer_ellipse.MajorRadius = COLLAR_MAJOR_RADIUS
    outer_ellipse.MinorRadius = COLLAR_MINOR_RADIUS
    outer_ellipse.Placement = App.Placement(
        App.Vector(-COLLAR_WIDTH / 2 - 10, 0, -COLLAR_MAJOR_RADIUS),
        App.Rotation(App.Vector(0, 1, 0), 90),
    )
    outer_ellipse.Label = 'outer_ellipse'
    doc.recompute()

    f = doc.addObject('Part::Extrusion', 'Extrude')
    f.Base = outer_ellipse  # .App.getDocument('Unnamed').getObject('Ellipse')
    f.DirMode = 'Normal'
    f.LengthFwd = COLLAR_WIDTH * 2
    f.Solid = True
    doc.recompute()

    # outer = doc.addObject('Part::Feature','Cylinder1').Shape=f.Shape

    outer = doc.addObject('Part::Feature', 'Cylinder1')
    outer.Label = 'Cylinder1'
    outer.Shape = Part.Solid(Part.Shell(f.Shape.Faces))

    doc.removeObject(outer_ellipse.Name)
    doc.removeObject(f.Name)
    doc.recompute()

    cut = doc.addObject('Part::Cut', 'Cut')
    doc.Cut.Base = arduinobox3
    doc.Cut.Tool = outer
    doc.recompute()

    arduinobox2a = doc.addObject('Part::Feature', 'arduinobox2a')
    arduinobox2a.Label = 'arduinobox2a'
    arduinobox2a.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject('Cut')
    doc.removeObject('arduinobox3')

    doc.removeObject('Cylinder1')
    doc.recompute()

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = arduinobox2a
    __fillets__ = []
    __fillets__.append((7, 2.00, 2.00))
    __fillets__.append((21, 2.00, 2.00))
    __fillets__.append((41, 2.00, 2.00))
    __fillets__.append((27, 2.00, 2.00))

    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    arduinobox2b = App.ActiveDocument.addObject(
        'Part::Feature', 'arduinobox2b'
    )
    arduinobox2b.Label = 'arduinobox2b'
    arduinobox2b.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject('Fillet')
    doc.removeObject('arduinobox2a')

    return arduinobox2b


def add_supports(doc, mainlid):
    support1 = doc.addObject('Part::Ellipsoid', 'support1')
    doc.support1.Radius1 = 3.10
    doc.support1.Radius2 = 4.00
    doc.support1.Radius3 = 10.00
    doc.support1.Angle1 = -90.00
    doc.support1.Angle2 = 90.00
    doc.support1.Angle3 = 360.00

    ANGLE = 66
    yloc = MAX_BASE_Y - INNER_EDGE_HEIGHT

    doc.support1.Placement = App.Placement(
        App.Vector(0.00, -53.7, yloc), App.Rotation(App.Vector(1, 0, 0), ANGLE)
    )
    doc.support1.Label = 'support1'

    support2 = doc.addObject('Part::Ellipsoid', 'support2')
    doc.support2.Radius1 = 3.10
    doc.support2.Radius2 = 4.00
    doc.support2.Radius3 = 10.00
    doc.support2.Angle1 = -90.00
    doc.support2.Angle2 = 90.00
    doc.support2.Angle3 = 360.00

    doc.support2.Placement = App.Placement(
        App.Vector(23.00, -53.7, yloc),
        App.Rotation(App.Vector(1, 0, 0), ANGLE),
    )
    doc.support2.Label = 'support2'

    support3 = doc.addObject('Part::Ellipsoid', 'support3')
    doc.support3.Radius1 = 3.10
    doc.support3.Radius2 = 4.00
    doc.support3.Radius3 = 10.00
    doc.support3.Angle1 = -90.00
    doc.support3.Angle2 = 90.00
    doc.support3.Angle3 = 360.00

    doc.support3.Placement = App.Placement(
        App.Vector(-23.00, -53.7, yloc),
        App.Rotation(App.Vector(1, 0, 0), ANGLE),
    )
    doc.support3.Label = 'support3'

    support4 = doc.addObject('Part::Ellipsoid', 'support4')
    doc.support4.Radius1 = 3.10
    doc.support4.Radius2 = 4.00
    doc.support4.Radius3 = 10.00
    doc.support4.Angle1 = -90.00
    doc.support4.Angle2 = 90.00
    doc.support4.Angle3 = 360.00

    doc.support4.Placement = App.Placement(
        App.Vector(0.00, 53.7, yloc), App.Rotation(App.Vector(1, 0, 0), -ANGLE)
    )
    doc.support4.Label = 'support4'

    support5 = doc.addObject('Part::Ellipsoid', 'support5')
    doc.support5.Radius1 = 3.10
    doc.support5.Radius2 = 4.00
    doc.support5.Radius3 = 10.00
    doc.support5.Angle1 = -90.00
    doc.support5.Angle2 = 90.00
    doc.support5.Angle3 = 360.00

    doc.support5.Placement = App.Placement(
        App.Vector(23.00, 53.7, yloc),
        App.Rotation(App.Vector(1, 0, 0), -ANGLE),
    )
    doc.support5.Label = 'support5'

    support6 = doc.addObject('Part::Ellipsoid', 'support6')
    doc.support6.Radius1 = 3.10
    doc.support6.Radius2 = 4.00
    doc.support6.Radius3 = 10.00
    doc.support6.Angle1 = -90.00
    doc.support6.Angle2 = 90.00
    doc.support6.Angle3 = 360.00

    doc.support6.Placement = App.Placement(
        App.Vector(-23.00, 53.7, yloc),
        App.Rotation(App.Vector(1, 0, 0), -ANGLE),
    )
    doc.support6.Label = 'support6'

    doc.recompute()

    fusion = doc.addObject('Part::MultiFuse', 'Fusion')
    doc.Fusion.Shapes = [
        mainlid,
        support1,
        support2,
        support3,
        support4,
        support5,
        support6,
    ]
    doc.recompute()

    lid2 = doc.addObject('Part::Feature', 'lid2')

    lid2.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject('Fusion')
    doc.removeObject(mainlid.Name)
    doc.removeObject('support1')
    doc.removeObject('support2')
    doc.removeObject('support3')
    doc.removeObject('support4')
    doc.removeObject('support5')
    doc.removeObject('support6')

    doc.recompute()
    return lid2


def cut_panel(doc, mainlid):
    PanelCutterBox = doc.addObject('Part::Box', 'PanelCutterBox')
    PanelCutterBox.Length = PANEL_LENGTH
    PanelCutterBox.Width = PANEL_WIDTH
    PanelCutterBox.Height = PANEL_HEIGHT
    PanelCutterBox.Placement = App.Placement(
        App.Vector(
            -(PANEL_LENGTH) / 2, -(PANEL_WIDTH) / 2, MAX_LID_Y - PANEL_HEIGHT
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    doc.recompute()
    cutpanelbox = doc.addObject('Part::Cut', 'cutpanelbox')
    cutpanelbox.Base = mainlid
    cutpanelbox.Tool = PanelCutterBox
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Label = 'mainlid2'
    mainlid2.Shape = Part.Solid(Part.Shell(cutpanelbox.Shape.Faces))

    doc.removeObject('cutpanelbox')
    doc.removeObject('PanelCutterBox')
    doc.removeObject(mainlid.Name)
    doc.recompute()

    return mainlid2


def add_marker(doc, mainlid):
    marker1 = doc.addObject('Part::Sphere', 'marker1')
    doc.marker1.Radius = 5.0

    doc.marker1.Placement = App.Placement(
        App.Vector(-30.0, 32.0, 5), App.Rotation(App.Vector(1, 0, 0), 0)
    )
    doc.marker1.Label = 'marker1'

    marker2 = doc.addObject('Part::Sphere', 'marker2')
    doc.marker2.Radius = 5.00

    doc.marker2.Placement = App.Placement(
        App.Vector(-30.0, -32.0, 5), App.Rotation(App.Vector(1, 0, 0), 0)
    )
    doc.marker2.Label = 'marker1'
    doc.recompute()

    fusion = doc.addObject('Part::MultiFuse', 'fusion')
    doc.fusion.Shapes = [mainlid, marker1]
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')

    mainlid2.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject(fusion.Label)

    doc.removeObject(mainlid.Name)

    fusion = doc.addObject('Part::MultiFuse', 'fusion')
    doc.fusion.Shapes = [mainlid2, marker2]
    doc.recompute()

    mainlid3 = doc.addObject('Part::Feature', 'mainlid3')

    mainlid3.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject(fusion.Label)

    doc.removeObject(mainlid2.Name)
    doc.removeObject('marker1')
    doc.removeObject('marker2')

    doc.recompute()
    return mainlid3


def cutout_lid(doc, mainlid):
    bwbwtol = BASE_TOP_WIDTH + 0.5
    blbwtol = BASE_TOP_LENGTH + 0.5
    bwtwtol = BASE_TOP_WIDTH + 0.5
    bltwtol = BASE_TOP_LENGTH + 0.5

    basecut = doc.addObject('Part::Wedge', 'basecut')
    basecut.Zmin = -bwbwtol / 2
    basecut.Xmin = -blbwtol / 2
    basecut.Z2min = -bwtwtol / 2
    basecut.X2min = -bltwtol / 2
    basecut.Zmax = bwbwtol / 2
    basecut.Xmax = blbwtol / 2
    basecut.Z2max = bwtwtol / 2
    basecut.X2max = bltwtol / 2

    basecut.Ymin = -100
    basecut.Ymax = (
        CAVITY_HEIGHT + FLOOR_THICKNESS - PLATE_THICKNESS
    )  # 10 mm for lipo, 3 for tolerance and lower into the casing

    basecut.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    cutchipbox = doc.addObject('Part::Cut', 'cutchipbox')
    cutchipbox.Base = mainlid
    cutchipbox.Tool = basecut
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Label = 'mainlid2'
    mainlid2.Shape = Part.Solid(Part.Shell(cutchipbox.Shape.Faces))

    doc.removeObject('basecut')

    doc.removeObject('cutchipbox')
    doc.removeObject(mainlid.Name)

    # chipcutter

    # first calculate the width of the outer box just below the panel
    ymin = MAX_BASE_Y - INNER_EDGE_HEIGHT
    ymax = MAX_LID_Y

    l1 = LID_BOTTOM_WIDTH
    l2 = LID_TOP_WIDTH

    ypanel = MAX_LID_Y - PANEL_HEIGHT - ROOF_THICKNESS

    # width at ypanel
    CUT_TOP_WIDTH = l1 + (l2 - l1) * (ypanel - ymin) / (ymax - ymin)
    CUT_TOP_WIDTH = CUT_TOP_WIDTH - 4  # add 2mm wall at each side

    ChipCutterBox1 = doc.addObject('Part::Wedge', 'ChipCutterBox1')
    ChipCutterBox1.Xmin = -(SEAL_LENGTH) / 2
    ChipCutterBox1.X2min = -(SEAL_LENGTH) / 2
    ChipCutterBox1.Xmax = (SEAL_LENGTH) / 2
    ChipCutterBox1.X2max = (SEAL_LENGTH) / 2

    ChipCutterBox1.Zmin = -(SEAL_WIDTH) / 2
    ChipCutterBox1.Z2min = -(CUT_TOP_WIDTH) / 2
    ChipCutterBox1.Zmax = (SEAL_WIDTH) / 2
    ChipCutterBox1.Z2max = (CUT_TOP_WIDTH) / 2

    ChipCutterBox1.Ymin = MAX_BASE_Y
    ChipCutterBox1.Ymax = CHIP_HEIGHT + FLOOR_THICKNESS - PLATE_THICKNESS
    ChipCutterBox1.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = ChipCutterBox1
    __fillets__ = []
    __fillets__.append((2, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((4, 2.00, 2.00))
    __fillets__.append((6, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    __fillets__.append((8, 2.00, 2.00))
    __fillets__.append((11, 2.00, 2.00))
    __fillets__.append((12, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    ChipCutterBox = App.ActiveDocument.addObject(
        'Part::Feature', 'ChipCutterBox'
    )
    ChipCutterBox.Label = 'ChipCutterBox'
    ChipCutterBox.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))  #

    doc.removeObject('Fillet')
    doc.removeObject('ChipCutterBox1')

    cutchipbox = doc.addObject('Part::Cut', 'cutchipbox')
    cutchipbox.Base = mainlid2
    cutchipbox.Tool = ChipCutterBox
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Label = 'mainlid'
    mainlid.Shape = Part.Solid(Part.Shell(cutchipbox.Shape.Faces))

    doc.removeObject('cutchipbox')
    doc.removeObject(mainlid2.Name)

    doc.removeObject('ChipCutterBox')

    return mainlid


def seal_protrusion(doc, mainlid):
    # SEAL_LENGTH = CHIP_LENGTH + 3
    # SEAL_WIDTH = CHIP_WIDTH + 2 + 3
    SealBox1 = doc.addObject('Part::Box', 'SealBox1')
    SealBox1.Length = SEAL_LENGTH
    SealBox1.Width = SEAL_WIDTH
    SealBox1.Height = SEAL_HEIGHT + 1
    SealBox1.Placement = App.Placement(
        App.Vector(
            -(SEAL_LENGTH) / 2, -(SEAL_WIDTH) / 2, MAX_BASE_Y - SEAL_HEIGHT
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = SealBox1
    __fillets__ = []
    __fillets__.append((1, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((5, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    SealBox1a = App.ActiveDocument.addObject('Part::Feature', 'SealBox1a')
    SealBox1a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject('Fillet')
    doc.removeObject('SealBox1')

    doc.recompute()

    SEAL_LENGTH2 = SEAL_LENGTH + 2.5
    SEAL_WIDTH2 = SEAL_WIDTH + 2.5
    SealBox2 = doc.addObject('Part::Box', 'SealBox2')
    SealBox2.Length = SEAL_LENGTH2
    SealBox2.Width = SEAL_WIDTH2
    SealBox2.Height = SEAL_HEIGHT + 1
    SealBox2.Placement = App.Placement(
        App.Vector(
            -(SEAL_LENGTH2) / 2, -(SEAL_WIDTH2) / 2, MAX_BASE_Y - SEAL_HEIGHT
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject('Part::Fillet', 'Fillet')
    fillet.Base = SealBox2
    __fillets__ = []
    __fillets__.append((1, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((5, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    SealBox2a = App.ActiveDocument.addObject('Part::Feature', 'SealBox2a')
    SealBox2a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject('Fillet')
    doc.removeObject('SealBox2')

    doc.recompute()

    sealboxcut = doc.addObject('Part::Cut', 'sealboxcut')
    sealboxcut.Base = SealBox2a
    sealboxcut.Tool = SealBox1a
    doc.recompute()

    sealbox = doc.addObject('Part::Feature', 'sealbox')
    sealbox.Label = 'sealbox'
    sealbox.Shape = Part.Solid(Part.Shell(sealboxcut.Shape.Faces))

    doc.removeObject('sealboxcut')
    doc.recompute()

    doc.removeObject('SealBox1a')
    doc.removeObject('SealBox2a')

    # stick seal and top together
    j = BOPTools.JoinFeatures.makeConnect(name='TopJoin')
    j.Objects = [mainlid, sealbox]
    j.Proxy.execute(j)
    j.purgeTouched()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Shape = Part.Solid(Part.Shell(doc.TopJoin.Shape.Faces))

    doc.removeObject('TopJoin')
    doc.removeObject(mainlid.Name)
    doc.removeObject('sealbox')
    return mainlid2


def solar_panel_wire(doc, mainlid):
    ellipse = doc.addObject('Part::Ellipsoid', 'Ellipsoid')
    doc.Ellipsoid.Radius1 = 20  # 15.00
    doc.Ellipsoid.Radius2 = 12.5
    doc.Ellipsoid.Radius3 = 7.5
    doc.Ellipsoid.Angle1 = -90.00
    doc.Ellipsoid.Angle2 = 90.00
    doc.Ellipsoid.Angle3 = 360.00
    doc.Ellipsoid.Placement = App.Placement(
        App.Vector(0.00, 15.50, MAX_LID_Y),
        App.Rotation(0.00, 0.00, 0.00, 1.00),
    )
    doc.Ellipsoid.Label = 'Ellipsoid'

    doc.recompute()

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid
    cut.Tool = ellipse
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid.Label)
    doc.removeObject(ellipse.Label)
    return mainlid2


def add_screw_holes(doc, mainlid):
    offset = 41.5
    yshift = offset * sin(radians(-SCREW_ANGLE))
    zshift = offset * cos(radians(-SCREW_ANGLE))

    ScrewCut1 = doc.addObject('Part::Cylinder', 'ScrewCut1')
    ScrewCut1.Height = 100 + MAX_LID_Y + PLATE_THICKNESS
    ScrewCut1.Radius = BIG_HEX_NUT_DIAM / 2

    ScrewCut1.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET + yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    ScrewCut2 = doc.addObject('Part::Cylinder', 'ScrewCut2')
    ScrewCut2.Height = 100 + MAX_LID_Y + PLATE_THICKNESS
    ScrewCut2.Radius = BIG_HEX_NUT_DIAM / 2
    ScrewCut2.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET + yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    ScrewCut3 = doc.addObject('Part::Cylinder', 'ScrewCut3')
    ScrewCut3.Height = 100 + MAX_LID_Y + PLATE_THICKNESS
    ScrewCut3.Radius = BIG_HEX_NUT_DIAM / 2
    ScrewCut3.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET - yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    ScrewCut4 = doc.addObject('Part::Cylinder', 'ScrewCut4')
    ScrewCut4.Height = 100 + MAX_LID_Y + PLATE_THICKNESS
    ScrewCut4.Radius = BIG_HEX_NUT_DIAM / 2
    ScrewCut4.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET - yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid
    cut.Tool = ScrewCut1
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Label = 'mainlid2'
    mainlid2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid2
    cut.Tool = ScrewCut2
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid2.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid
    cut.Tool = ScrewCut3
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid2
    cut.Tool = ScrewCut4
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid2.Label)

    doc.removeObject(ScrewCut1.Label)
    doc.removeObject(ScrewCut2.Label)
    doc.removeObject(ScrewCut3.Label)
    doc.removeObject(ScrewCut4.Label)

    # hex nuts for the lipo base

    offset = 30
    yshift = offset * sin(radians(-SCREW_ANGLE))
    zshift = offset * cos(radians(-SCREW_ANGLE))

    ScrewCut1 = doc.addObject('Part::Cylinder', 'ScrewCut1')
    ScrewCut1.Height = 15
    ScrewCut1.Radius = HEX_NUT_DIAM / 2

    ScrewCut1.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET + yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    # ScrewCut1.Placement = App.Placement(App.Vector(-10.0,-10.0,0),App.Rotation(App.Vector(1,0,0),56))

    ScrewCut2 = doc.addObject('Part::Cylinder', 'ScrewCut2')
    ScrewCut2.Height = 15
    ScrewCut2.Radius = HEX_NUT_DIAM / 2
    ScrewCut2.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET + yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    ScrewCut3 = doc.addObject('Part::Cylinder', 'ScrewCut3')
    ScrewCut3.Height = 15
    ScrewCut3.Radius = HEX_NUT_DIAM / 2
    ScrewCut3.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET - yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    ScrewCut4 = doc.addObject('Part::Cylinder', 'ScrewCut4')
    ScrewCut4.Height = 15
    ScrewCut4.Radius = HEX_NUT_DIAM / 2
    ScrewCut4.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET - yshift,
            -50 + zshift,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    # START OF HOLES INTO BASE

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid
    cut.Tool = ScrewCut1
    doc.recompute()
    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Label = 'mainlid2'
    mainlid2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid2
    cut.Tool = ScrewCut2
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid2.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid
    cut.Tool = ScrewCut3
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid.Label)

    cut = doc.addObject('Part::Cut', 'Cut')
    cut.Base = mainlid2
    cut.Tool = ScrewCut4
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(mainlid2.Label)

    doc.removeObject(ScrewCut1.Label)
    doc.removeObject(ScrewCut2.Label)
    doc.removeObject(ScrewCut3.Label)
    doc.removeObject(ScrewCut4.Label)

    doc.recompute()

    return mainlid


def add_gripper(doc, mainlid):
    GRIPPER_WIDTH = 2
    GRIPPER_HEIGHT = 19.3
    gripperbox1 = doc.addObject('Part::Box', 'gripperbox1')
    gripperbox1.Length = 10
    gripperbox1.Width = GRIPPER_WIDTH
    gripperbox1.Height = GRIPPER_HEIGHT

    gripperbox1.Placement = App.Placement(
        App.Vector(
            -CHIP_LENGTH / 2 + 7,
            CHIP_WIDTH / 2 - GRIPPER_WIDTH - 1,
            MAX_BASE_Y - CAVITY_HEIGHT + PCB_HEIGHT,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    GRIPPER_WIDTH = 6
    GRIPPER_HEIGHT = 8
    gripperbox1a = doc.addObject('Part::Box', 'gripperbox1a')
    gripperbox1a.Length = 10
    gripperbox1a.Width = GRIPPER_WIDTH
    gripperbox1a.Height = GRIPPER_HEIGHT

    gripperbox1a.Placement = App.Placement(
        App.Vector(
            -CHIP_LENGTH / 2 + 7,
            CHIP_WIDTH / 2 - 3,
            MAX_BASE_Y,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    GRIPPER_WIDTH = 4
    GRIPPER_HEIGHT = 5
    gripperbox1b = doc.addObject('Part::Box', 'gripperbox1b')
    gripperbox1b.Length = 10
    gripperbox1b.Width = GRIPPER_WIDTH
    gripperbox1b.Height = GRIPPER_HEIGHT

    gripperbox1b.Placement = App.Placement(
        App.Vector(
            -CHIP_LENGTH / 2 + 7,
            CHIP_WIDTH / 2 - 3,
            MAX_BASE_Y + 8,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    fusion = doc.addObject('Part::MultiFuse', 'Fusion')
    doc.Fusion.Shapes = [gripperbox1, gripperbox1a, gripperbox1b]
    doc.recompute()

    gripper1 = doc.addObject('Part::Feature', 'gripper1')
    gripper1.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject('Fusion')
    doc.removeObject(gripperbox1.Name)
    doc.removeObject(gripperbox1a.Name)
    doc.removeObject(gripperbox1b.Name)

    GRIPPER_WIDTH = 2
    GRIPPER_HEIGHT = 20.5
    gripperbox2 = doc.addObject('Part::Box', 'gripperbox2')
    gripperbox2.Length = 3
    gripperbox2.Width = GRIPPER_WIDTH
    gripperbox2.Height = GRIPPER_HEIGHT

    gripperbox2.Placement = App.Placement(
        App.Vector(
            0,
            -CHIP_WIDTH / 2 + GRIPPER_WIDTH,
            MAX_BASE_Y - CAVITY_HEIGHT + PCB_HEIGHT,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    GRIPPER_WIDTH = 7
    GRIPPER_HEIGHT = 8.0
    gripperbox2a = doc.addObject('Part::Box', 'gripperbox2a')
    gripperbox2a.Length = 3
    gripperbox2a.Width = GRIPPER_WIDTH
    gripperbox2a.Height = GRIPPER_HEIGHT

    gripperbox2a.Placement = App.Placement(
        App.Vector(
            0,
            -CHIP_WIDTH / 2 - 3,
            MAX_BASE_Y,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    GRIPPER_WIDTH = 2.2
    GRIPPER_HEIGHT = 6.2
    gripperbox2b = doc.addObject('Part::Box', 'gripperbox2b')
    gripperbox2b.Length = 3
    gripperbox2b.Width = GRIPPER_WIDTH
    gripperbox2b.Height = GRIPPER_HEIGHT

    gripperbox2b.Placement = App.Placement(
        App.Vector(
            0,
            -CHIP_WIDTH / 2 - 0.2,
            MAX_BASE_Y + 8,
        ),
        App.Rotation(App.Vector(1, 0, 0), 0),
    )

    fusion = doc.addObject('Part::MultiFuse', 'Fusion')
    doc.Fusion.Shapes = [gripperbox2, gripperbox2a, gripperbox2b]
    doc.recompute()

    gripper2 = doc.addObject('Part::Feature', 'gripper2')
    gripper2.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject('Fusion')
    doc.removeObject(gripperbox2.Name)
    doc.removeObject(gripperbox2a.Name)
    doc.removeObject(gripperbox2b.Name)

    doc.recompute()

    fusion = doc.addObject('Part::MultiFuse', 'Fusion')
    doc.Fusion.Shapes = [mainlid, gripper1, gripper2]
    doc.recompute()

    mainlid2 = doc.addObject('Part::Feature', 'mainlid2')
    mainlid2.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject('Fusion')
    doc.removeObject(mainlid.Name)
    doc.removeObject('gripper1')
    doc.removeObject('gripper2')

    return mainlid2


def build_top_case(doc):
    toplid = build_lid_top(doc)
    baselid = build_lid_base(doc)

    fusion = doc.addObject('Part::MultiFuse', 'Fusion')
    doc.Fusion.Shapes = [toplid, baselid]
    doc.recompute()

    mainlid = doc.addObject('Part::Feature', 'mainlid')
    mainlid.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject('Fusion')
    doc.removeObject(toplid.Name)
    doc.removeObject(baselid.Name)

    doc.recompute()

    mainlid = add_supports(doc, mainlid)
    mainlid = cut_panel(doc, mainlid)
    mainlid = add_marker(doc, mainlid)
    mainlid = cutout_lid(doc, mainlid)
    mainlid = seal_protrusion(doc, mainlid)
    mainlid = solar_panel_wire(doc, mainlid)
    mainlid = add_screw_holes(doc, mainlid)
    mainlid = add_gripper(doc, mainlid)
