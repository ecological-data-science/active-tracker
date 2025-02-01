from FreeCAD import Base
import FreeCAD as App
import BOPTools.JoinFeatures
import Part
import Part, PartGui
import Mesh
import sys


from base_case import build_base_case

# this is the maximum height needed inside the case
MAX_HEIGHT = 18

# this is the thickness of the curved base
PLATE_THICKNESS = 10

# this is the thickness of the base once the inset has been cut out
FLOOR_THICKNESS = 3


# the top of the base is a location 0,0,0
# the lowest edge of the base is at -10 in the centre

# the total height is 3 + 18 + 3 + 3 = 27 which is floor thickness + the cavity height + the thickness of the lid + the thickness of the panel

COLLAR_MAJOR_RADIUS = 350
COLLAR_MINOR_RADIUS = 140
COLLAR_WIDTH = 50

PLATE_LENGTH = 128 + 20 + 20


LIPO_WIDTH = 60 + 2
LIPO_LENGTH = 40
CAVITY_HEIGHT = 10


CHIP_WIDTH = 88
CHIP_LENGTH = 48
CHIP_HEIGHT = 13


SEAL_LENGTH = CHIP_LENGTH + 3 + 4
SEAL_WIDTH = CHIP_WIDTH + 5


bwb = SEAL_WIDTH + 4 + 4  # battery length bottom
blb = 50  # battery width bottom
bwt = SEAL_WIDTH + 4 + 4  # battery length top
blt = SEAL_LENGTH + 4 + 4  # battery width top


PANEL_WIDTH = 65
PANEL_LENGTH = 65
PANEL_HEIGHT = 3

slb = blt + 4.5
swb = bwt + 8
slt2 = PANEL_LENGTH + 6

swt2 = PANEL_WIDTH + 14 + 6  # 24 +GROW

SCREW_LOCATION_W = 83  # PANEL_WIDTH + 13 # 24 +GROW
SCREW_LOCATION_L = 63  # PANEL_LENGTH + 8


def build_lid_top(doc):
    ymin = (
        CAVITY_HEIGHT + 3 - PLATE_THICKNESS
    )  # at this point the wedge must be wb x lb
    ymax = MAX_HEIGHT

    arduinobox1 = doc.addObject("Part::Wedge", "arduinobox1")
    arduinobox1.Zmin = -swb / 2
    arduinobox1.Xmin = -slb / 2
    arduinobox1.Z2min = -swt2 / 2
    arduinobox1.X2min = -slt2 / 2
    arduinobox1.Zmax = swb / 2
    arduinobox1.Xmax = slb / 2
    arduinobox1.Z2max = swt2 / 2
    arduinobox1.X2max = slt2 / 2

    arduinobox1.Ymin = ymin
    arduinobox1.Ymax = MAX_HEIGHT

    arduinobox1.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    fillet = doc.addObject("Part::Fillet", "Fillet")
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

    arduinobox1a = App.ActiveDocument.addObject("Part::Feature", "arduinobox1a")
    arduinobox1a.Label = "arduinobox1a"
    arduinobox1a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject("Fillet")
    doc.removeObject("arduinobox1")

    return arduinobox1a


def build_lid_base(doc):
    ymin = -90  # at this point the wedge must be wb x lb
    ymax = CAVITY_HEIGHT + 3 - PLATE_THICKNESS

    lower_width = 18  # 22.5 #25.5
    lower_length = 140 + 10 + 6

    arduinobox2 = doc.addObject("Part::Wedge", "arduinobox2")
    arduinobox2.Zmin = -lower_length / 2
    arduinobox2.Xmin = -lower_width / 2
    arduinobox2.Z2min = -swb / 2
    arduinobox2.X2min = -slb / 2
    arduinobox2.Zmax = lower_length / 2
    arduinobox2.Xmax = lower_width / 2
    arduinobox2.Z2max = swb / 2
    arduinobox2.X2max = slb / 2

    arduinobox2.Ymin = ymin
    arduinobox2.Ymax = ymax + 0.01

    arduinobox2.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    CUT_LIPO_BOX_HEIGHT2 = 50
    lipobox2 = doc.addObject("Part::Box", "lipobox2")
    LIPO_BOX_HEIGHT2 = 2
    LIPO_BOX_HEIGHT = 3
    lipobox2.Length = blt + 4
    lipobox2.Width = bwt + 0.5
    lipobox2.Height = CUT_LIPO_BOX_HEIGHT2
    lipobox2.Placement = App.Placement(
        App.Vector(
            -(blt + 4) / 2,
            -(bwt + 0.5) / 2,
            -LIPO_BOX_HEIGHT - CUT_LIPO_BOX_HEIGHT2 + 3,
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject("Part::Fillet", "Fillet")
    fillet.Base = lipobox2
    __fillets__ = []
    __fillets__.append((10, 1.50, 1.50))
    __fillets__.append((12, 1.50, 1.50))

    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    lipobox3 = App.ActiveDocument.addObject("Part::Feature", "lipobox3")
    lipobox3.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject("Fillet")
    doc.removeObject("lipobox2")

    doc.recompute()

    cutchipbox2 = doc.addObject("Part::Cut", "cutchipbox2")
    cutchipbox2.Base = arduinobox2
    cutchipbox2.Tool = lipobox3
    doc.recompute()

    arduinobox3 = doc.addObject("Part::Feature", "arduinobox3")
    arduinobox3.Label = "arduinobox3"
    arduinobox3.Shape = Part.Solid(Part.Shell(cutchipbox2.Shape.Faces))

    doc.removeObject("lipobox3")
    doc.removeObject("cutchipbox2")
    doc.removeObject("arduinobox2")

    # MAKE THE OUTER ELLIPSE FOR COLLAR
    outer_ellipse = doc.addObject("Part::Ellipse", "outer_ellipse")
    outer_ellipse.MajorRadius = COLLAR_MAJOR_RADIUS
    outer_ellipse.MinorRadius = COLLAR_MINOR_RADIUS
    outer_ellipse.Placement = App.Placement(
        App.Vector(-COLLAR_WIDTH / 2 - 10, 0, -COLLAR_MAJOR_RADIUS),
        App.Rotation(App.Vector(0, 1, 0), 90),
    )
    outer_ellipse.Label = "outer_ellipse"
    doc.recompute()

    f = doc.addObject("Part::Extrusion", "Extrude")
    f.Base = outer_ellipse  # .App.getDocument('Unnamed').getObject('Ellipse')
    f.DirMode = "Normal"
    f.LengthFwd = COLLAR_WIDTH * 2
    f.Solid = True
    doc.recompute()

    # outer = doc.addObject('Part::Feature','Cylinder1').Shape=f.Shape

    outer = doc.addObject("Part::Feature", "Cylinder1")
    outer.Label = "Cylinder1"
    outer.Shape = Part.Solid(Part.Shell(f.Shape.Faces))

    doc.removeObject(outer_ellipse.Name)
    doc.removeObject(f.Name)
    doc.recompute()

    cut = doc.addObject("Part::Cut", "Cut")
    doc.Cut.Base = arduinobox3
    doc.Cut.Tool = outer
    doc.recompute()

    arduinobox2a = doc.addObject("Part::Feature", "arduinobox2a")
    arduinobox2a.Label = "arduinobox2a"
    arduinobox2a.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject("Cut")
    doc.removeObject("arduinobox3")

    doc.removeObject("Cylinder1")
    doc.recompute()

    fillet = doc.addObject("Part::Fillet", "Fillet")
    fillet.Base = arduinobox2a
    __fillets__ = []
    __fillets__.append((1, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((13, 2.00, 2.00))
    __fillets__.append((17, 2.00, 2.00))

    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    arduinobox2b = App.ActiveDocument.addObject("Part::Feature", "arduinobox2b")
    arduinobox2b.Label = "arduinobox2b"
    arduinobox2b.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject("Fillet")
    doc.removeObject("arduinobox2a")

    return arduinobox2b


def add_supports(doc, mainlid):
    support1 = doc.addObject("Part::Ellipsoid", "support1")
    doc.support1.Radius1 = 3.10
    doc.support1.Radius2 = 4.00
    doc.support1.Radius3 = 10.00
    doc.support1.Angle1 = -90.00
    doc.support1.Angle2 = 90.00
    doc.support1.Angle3 = 360.00

    doc.support1.Placement = App.Placement(
        App.Vector(0.00, -53.0, 1), App.Rotation(App.Vector(1, 0, 0), 65)
    )
    doc.support1.Label = "support1"

    support2 = doc.addObject("Part::Ellipsoid", "support2")
    doc.support2.Radius1 = 3.10
    doc.support2.Radius2 = 4.00
    doc.support2.Radius3 = 10.00
    doc.support2.Angle1 = -90.00
    doc.support2.Angle2 = 90.00
    doc.support2.Angle3 = 360.00

    doc.support2.Placement = App.Placement(
        App.Vector(20.00, -53.0, 1), App.Rotation(App.Vector(1, 0, 0), 65)
    )
    doc.support2.Label = "support2"

    support3 = doc.addObject("Part::Ellipsoid", "support3")
    doc.support3.Radius1 = 3.10
    doc.support3.Radius2 = 4.00
    doc.support3.Radius3 = 10.00
    doc.support3.Angle1 = -90.00
    doc.support3.Angle2 = 90.00
    doc.support3.Angle3 = 360.00

    doc.support3.Placement = App.Placement(
        App.Vector(-20.00, -53.0, 1), App.Rotation(App.Vector(1, 0, 0), 65)
    )
    doc.support3.Label = "support3"

    support4 = doc.addObject("Part::Ellipsoid", "support4")
    doc.support4.Radius1 = 3.10
    doc.support4.Radius2 = 4.00
    doc.support4.Radius3 = 10.00
    doc.support4.Angle1 = -90.00
    doc.support4.Angle2 = 90.00
    doc.support4.Angle3 = 360.00

    doc.support4.Placement = App.Placement(
        App.Vector(0.00, 53.0, 1), App.Rotation(App.Vector(1, 0, 0), -65)
    )
    doc.support4.Label = "support4"

    support5 = doc.addObject("Part::Ellipsoid", "support5")
    doc.support5.Radius1 = 3.10
    doc.support5.Radius2 = 4.00
    doc.support5.Radius3 = 10.00
    doc.support5.Angle1 = -90.00
    doc.support5.Angle2 = 90.00
    doc.support5.Angle3 = 360.00

    doc.support5.Placement = App.Placement(
        App.Vector(20.00, 53.0, 1), App.Rotation(App.Vector(1, 0, 0), -65)
    )
    doc.support5.Label = "support5"

    support6 = doc.addObject("Part::Ellipsoid", "support6")
    doc.support6.Radius1 = 3.10
    doc.support6.Radius2 = 4.00
    doc.support6.Radius3 = 10.00
    doc.support6.Angle1 = -90.00
    doc.support6.Angle2 = 90.00
    doc.support6.Angle3 = 360.00

    doc.support6.Placement = App.Placement(
        App.Vector(-20.00, 53.0, 1), App.Rotation(App.Vector(1, 0, 0), -65)
    )
    doc.support6.Label = "support6"

    doc.recompute()

    fusion = doc.addObject("Part::MultiFuse", "Fusion")
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

    lid2 = doc.addObject("Part::Feature", "lid2")

    lid2.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject("Fusion")
    doc.removeObject(mainlid.Name)
    doc.removeObject("support1")
    doc.removeObject("support2")
    doc.removeObject("support3")
    doc.removeObject("support4")
    doc.removeObject("support5")
    doc.removeObject("support6")

    doc.recompute()
    return lid2


def cut_panel(doc, mainlid):
    PanelCutterBox = doc.addObject("Part::Box", "PanelCutterBox")
    PanelCutterBox.Length = PANEL_LENGTH
    PanelCutterBox.Width = PANEL_WIDTH
    PanelCutterBox.Height = PANEL_HEIGHT + 1
    PanelCutterBox.Placement = App.Placement(
        App.Vector(
            -(PANEL_LENGTH) / 2, -(PANEL_WIDTH) / 2, MAX_HEIGHT - PANEL_HEIGHT - 1
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    doc.recompute()
    cutpanelbox = doc.addObject("Part::Cut", "cutpanelbox")
    cutpanelbox.Base = mainlid
    cutpanelbox.Tool = PanelCutterBox
    doc.recompute()

    mainlid2 = doc.addObject("Part::Feature", "mainlid2")
    mainlid2.Label = "mainlid2"
    mainlid2.Shape = Part.Solid(Part.Shell(cutpanelbox.Shape.Faces))

    doc.removeObject("cutpanelbox")
    doc.removeObject("PanelCutterBox")
    doc.removeObject(mainlid.Name)
    doc.recompute()

    return mainlid2


def build_top_case(doc):
    toplid = build_lid_top(doc)
    baselid = build_lid_base(doc)

    fusion = doc.addObject("Part::MultiFuse", "Fusion")
    doc.Fusion.Shapes = [toplid, baselid]
    doc.recompute()

    mainlid = doc.addObject("Part::Feature", "mainlid")
    mainlid.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject("Fusion")
    doc.removeObject(toplid.Name)
    doc.removeObject(baselid.Name)

    doc.recompute()

    mainlid = add_supports(doc, mainlid)

    mainlid = cut_panel(doc, mainlid)
