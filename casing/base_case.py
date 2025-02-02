from FreeCAD import Base
import FreeCAD as App
import BOPTools.JoinFeatures
import Part
import Part, PartGui
import Mesh
import sys

from math import sin, cos, radians
from dimensions import *

import importlib

importlib.reload(sys.modules["dimensions"])


def create_cylinder(doc, offset):
    ellipse = doc.addObject("Part::Ellipse", "ellipse")
    ellipse.MajorRadius = COLLAR_MAJOR_RADIUS - offset
    ellipse.MinorRadius = COLLAR_MINOR_RADIUS - offset
    ellipse.Placement = App.Placement(
        App.Vector(-COLLAR_WIDTH / 2 - offset / 2, 0, -COLLAR_MAJOR_RADIUS),
        App.Rotation(App.Vector(0, 1, 0), 90),
    )
    ellipse.Label = "ellipse"
    doc.recompute()

    f = doc.addObject("Part::Extrusion", "Extrude")
    f.Base = ellipse
    f.DirMode = "Normal"
    f.LengthFwd = COLLAR_WIDTH + offset
    f.Solid = True
    doc.recompute()

    cylinder = doc.addObject("Part::Feature", "Cylinder2")
    cylinder.Label = "Cylinder2"
    cylinder.Shape = Part.Solid(Part.Shell(f.Shape.Faces))
    doc.removeObject(ellipse.Name)
    doc.removeObject(f.Name)
    doc.recompute()
    return cylinder


def create_curved_base(doc):
    """
    This function creates the curved base of the case by creating two cylinders and cutting one from the other.
    The offset parameter is used to create the inner cylinder.

    """

    outer = create_cylinder(doc, offset=0)
    inner = create_cylinder(doc, offset=PLATE_THICKNESS)

    cut = doc.addObject("Part::Cut", "Cut")
    doc.Cut.Base = outer
    doc.Cut.Tool = inner
    doc.recompute()

    full_cylinder = doc.addObject("Part::Feature", "Cut_solid")
    full_cylinder.Label = "Cut (Solid)"
    full_cylinder.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Name)
    doc.removeObject(outer.Label)
    doc.removeObject(inner.Label)
    doc.recompute()

    # next retain only the section of the cylinder that is needed to mount the device
    ymin, ymax = -90, 3
    lower_width, lower_length = 18, 156
    upper_width, upper_length = 68, 110

    retainer = doc.addObject("Part::Wedge", "retainer")
    retainer.Zmin, retainer.Xmin = -lower_length / 2, -lower_width / 2
    retainer.Z2min, retainer.X2min = -upper_length / 2, -upper_width / 2
    retainer.Zmax, retainer.Xmax = lower_length / 2, lower_width / 2
    retainer.Z2max, retainer.X2max = upper_length / 2, upper_width / 2
    retainer.Ymin, retainer.Ymax = ymin, ymax

    retainer.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    common = doc.addObject("Part::MultiCommon", "common")
    doc.common.Shapes = [
        doc.retainer,
        full_cylinder,
    ]

    doc.recompute()

    curved_base = doc.addObject("Part::Feature", "curved_base")
    curved_base.Shape = Part.Solid(Part.Shell(common.Shape.Faces))

    doc.removeObject(common.Name)
    doc.removeObject(full_cylinder.Name)
    doc.removeObject(retainer.Name)
    return curved_base


def pcb_platform(doc, curved_base):
    lipobox1 = doc.addObject("Part::Box", "lipobox1")
    lipobox1.Length = BASE_TOP_LENGTH
    lipobox1.Width = BASE_TOP_WIDTH
    lipobox1.Height = INNER_EDGE_HEIGHT

    lipobox1.Placement = App.Placement(
        App.Vector(
            -(BASE_TOP_LENGTH) / 2,
            -(BASE_TOP_WIDTH) / 2,
            MAX_BASE_Y - INNER_EDGE_HEIGHT,
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    lipobox2 = doc.addObject("Part::Box", "lipobox2")
    lipobox2.Length = BASE_TOP_LENGTH + 2
    lipobox2.Width = BASE_TOP_WIDTH
    lipobox2.Height = OUTER_EDGE_HEIGHT

    lipobox2.Placement = App.Placement(
        App.Vector(
            -(BASE_TOP_LENGTH + 2) / 2,
            -(BASE_TOP_WIDTH) / 2,
            MAX_BASE_Y - INNER_EDGE_HEIGHT - OUTER_EDGE_HEIGHT,
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    # lipoboxwedge = doc.addObject("Part::Wedge", "lipoboxwedge")
    # lipoboxwedge.Zmin = -BASE_TOP_WIDTH / 2
    # lipoboxwedge.Xmin = -COLLAR_WIDTH / 2
    # lipoboxwedge.Z2min = -BASE_TOP_WIDTH / 2
    # lipoboxwedge.X2min = -BASE_TOP_LENGTH / 2 - 1
    # lipoboxwedge.Zmax = BASE_TOP_WIDTH / 2
    # lipoboxwedge.Xmax = COLLAR_WIDTH / 2
    # lipoboxwedge.Z2max = BASE_TOP_WIDTH / 2
    # lipoboxwedge.X2max = BASE_TOP_LENGTH / 2 + 1
    #
    # lipoboxwedge.Ymin = -PLATE_THICKNESS  # -32.88
    # lipoboxwedge.Ymax = (
    #     -INNER_EDGE_HEIGHT - OUTER_EDGE_HEIGHT + 3.00
    # )  # CAVITY_HEIGHT + 3 - PLATE_THICKNESS  # 10 mm for lipo, 3 for tolerance and lower into the casing
    #
    # lipoboxwedge.Placement = App.Placement(
    #     App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    # )

    surfacewedge = doc.addObject("Part::Wedge", "surfacewedge")
    surfacewedge.Zmin = -BASE_TOP_WIDTH / 2
    surfacewedge.Xmin = -COLLAR_WIDTH / 2
    surfacewedge.Z2min = -BASE_TOP_WIDTH / 2
    surfacewedge.X2min = -BASE_TOP_LENGTH / 2 - 1
    surfacewedge.Zmax = -BASE_TOP_WIDTH / 2 + 2
    surfacewedge.Xmax = COLLAR_WIDTH / 2
    surfacewedge.Z2max = -BASE_TOP_WIDTH / 2 + 2
    surfacewedge.X2max = BASE_TOP_LENGTH / 2 + 1

    surfacewedge.Ymin = -30
    surfacewedge.Ymax = (
        -INNER_EDGE_HEIGHT - OUTER_EDGE_HEIGHT + MAX_BASE_Y
    )  # CAVITY_HEIGHT + 3 - PLATE_THICKNESS  # 10 mm for lipo, 3 for tolerance and lower into the casing

    surfacewedge.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    surfacewedge2 = doc.addObject("Part::Wedge", "surfacewedge2")
    surfacewedge2.Zmin = BASE_TOP_WIDTH / 2 - 2
    surfacewedge2.Xmin = -COLLAR_WIDTH / 2
    surfacewedge2.Z2min = BASE_TOP_WIDTH / 2 - 2
    surfacewedge2.X2min = -BASE_TOP_LENGTH / 2 - 1
    surfacewedge2.Zmax = BASE_TOP_WIDTH / 2
    surfacewedge2.Xmax = COLLAR_WIDTH / 2
    surfacewedge2.Z2max = BASE_TOP_WIDTH / 2
    surfacewedge2.X2max = BASE_TOP_LENGTH / 2 + 1

    surfacewedge2.Ymin = -30
    surfacewedge2.Ymax = (
        -INNER_EDGE_HEIGHT - OUTER_EDGE_HEIGHT + MAX_BASE_Y
    )  # CAVITY_HEIGHT + 3 - PLATE_THICKNESS  # 10 mm for lipo, 3 for tolerance and lower into the casing
    surfacewedge2.Placement = App.Placement(
        App.Vector(0, 0, 0), App.Rotation(App.Vector(1, 0, 0), 90)
    )

    LipoFloor = doc.addObject("Part::Box", "LipoFloor")
    LipoFloor.Length = COLLAR_WIDTH
    LipoFloor.Width = BASE_TOP_WIDTH
    LipoFloor.Height = 30 + MAX_BASE_Y  # THICKNESS OF BASE AT THINNEST POINT

    ymin = -30
    LipoFloor.Placement = App.Placement(
        App.Vector(-(COLLAR_WIDTH) / 2, -(BASE_TOP_WIDTH) / 2, ymin),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    j = BOPTools.JoinFeatures.makeConnect(name="LipoFloorJoin")
    j.Objects = [doc.lipobox1, doc.lipobox2, doc.surfacewedge, doc.surfacewedge2]
    j.Proxy.execute(j)
    j.purgeTouched()
    for obj in j.ViewObject.Proxy.claimChildren():
        obj.ViewObject.hide()

    lipocase = doc.addObject("Part::Feature", "lipocase")
    lipocase.Label = "lipocase"
    lipocase.Shape = Part.Solid(Part.Shell(doc.LipoFloorJoin.Shape.Faces))

    doc.removeObject("LipoFloorJoin")
    doc.removeObject("lipobox1")
    doc.removeObject("lipobox2")
    # doc.removeObject("lipoboxwedge")
    doc.removeObject("surfacewedge")
    doc.removeObject("surfacewedge2")

    doc.recompute()

    fusion = doc.addObject("Part::MultiFuse", "Fusion")
    doc.Fusion.Shapes = [doc.LipoFloor, doc.lipocase]
    doc.recompute()

    baseholder = doc.addObject("Part::Feature", "baseholder")
    baseholder.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.recompute()
    doc.removeObject(fusion.Label)
    doc.removeObject(LipoFloor.Label)
    doc.removeObject(lipocase.Label)

    inner = create_cylinder(doc, offset=PLATE_THICKNESS)
    cut = doc.addObject("Part::Cut", "Cut")
    doc.Cut.Base = baseholder
    doc.Cut.Tool = inner
    doc.recompute()

    FullBase = doc.addObject("Part::Feature", "FullBase")
    FullBase.Label = "FullBase"
    FullBase.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))
    doc.removeObject("Cut")
    doc.removeObject("baseholder")
    doc.removeObject(inner.Name)
    doc.recompute()
    fusion = doc.addObject("Part::MultiFuse", "Fusion")
    doc.Fusion.Shapes = [FullBase, curved_base]
    doc.recompute()

    baseholder = doc.addObject("Part::Feature", "baseholder")

    baseholder.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject(fusion.Name)
    doc.removeObject(FullBase.Name)
    doc.removeObject(curved_base.Name)
    cutbox = doc.addObject("Part::Box", "cutbox")
    cutbox.Length = BASE_TOP_LENGTH
    cutbox.Width = BASE_TOP_WIDTH
    cutbox.Height = 100

    cutbox.Placement = App.Placement(
        App.Vector(
            -(BASE_TOP_LENGTH) / 2,
            -(BASE_TOP_WIDTH) / 2,
            MAX_BASE_Y,
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    # cut using the box
    cut = doc.addObject("Part::Cut", "Cut")
    cut.Base = baseholder
    cut.Tool = cutbox
    doc.recompute()

    baseholder2 = doc.addObject("Part::Feature", "baseholder")
    baseholder2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cutbox.Name)
    doc.removeObject(cut.Name)
    doc.removeObject(baseholder.Name)

    return baseholder2


def add_surface(doc, baseholder):
    # create the smooth surface edges

    surf1 = doc.addObject("Surface::Filling", "surf1")
    surf1.BoundaryEdges = [
        (baseholder, "Edge3"),
        (baseholder, "Edge4"),
        (baseholder, "Edge5"),
        (baseholder, "Edge12"),
        (baseholder, "Edge28"),
        (baseholder, "Edge30"),
    ]

    surf2 = doc.addObject("Surface::Filling", "surf2")
    surf2.BoundaryEdges = [
        (baseholder, "Edge19"),
        (baseholder, "Edge47"),
        (baseholder, "Edge48"),
        (baseholder, "Edge80"),
        (baseholder, "Edge85"),
        (baseholder, "Edge89"),
    ]

    exsurf1 = doc.addObject("Part::Extrusion", "exsurf1")
    # f = doc.getObject('Extrude')
    exsurf1.Base = surf1  # App.getDocument('Unnamed').getObject('Surface')
    exsurf1.DirMode = "Custom"
    exsurf1.Dir = App.Vector(1.0, 0.0, 0.0)
    exsurf1.DirLink = None
    exsurf1.LengthFwd = 0.0
    exsurf1.LengthRev = 10.0
    doc.recompute()

    exsurf2 = doc.addObject("Part::Extrusion", "exsurf2")
    exsurf2.Base = surf2  # App.getDocument('Unnamed').getObject('Surface')
    exsurf2.DirMode = "Custom"
    exsurf2.Dir = App.Vector(1.0, 0.0, 0.0)
    exsurf2.DirLink = None
    exsurf2.LengthFwd = 10.0
    exsurf2.LengthRev = 0.0
    doc.recompute()

    fusion = doc.addObject("Part::MultiFuse", "Fusion")
    doc.Fusion.Shapes = [baseholder, exsurf1, exsurf2]
    doc.recompute()

    newbaseholder = doc.addObject("Part::Feature", "newbaseholder")

    newbaseholder.Shape = Part.Solid(Part.Shell(fusion.Shape.Faces))

    doc.removeObject(fusion.Name)
    doc.removeObject(baseholder.Name)
    doc.removeObject(exsurf1.Name)
    doc.removeObject(exsurf2.Name)
    doc.removeObject(surf1.Name)
    doc.removeObject(surf2.Name)
    doc.recompute()
    return newbaseholder


def add_groove(doc, baseholder):
    # this is the location of the top surface of the chip box
    # the top of the curved plate is at 0,0,0 so starting at the bottom of the plate
    # we have 3 for the wall thickness and then the cavity height
    CHIPBOX_MIN = CAVITY_HEIGHT + 3 - PLATE_THICKNESS

    INPUT_LENGTH = SEAL_LENGTH - GROOVE_TOLERANCE
    INPUT_WIDTH = SEAL_WIDTH - GROOVE_TOLERANCE
    InputBox1 = doc.addObject("Part::Box", "InputBox1")
    InputBox1.Length = INPUT_LENGTH
    InputBox1.Width = INPUT_WIDTH
    InputBox1.Height = GROOVE_HEIGHT
    InputBox1.Placement = App.Placement(
        App.Vector(-(INPUT_LENGTH) / 2, -(INPUT_WIDTH) / 2, MAX_BASE_Y - GROOVE_HEIGHT),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject("Part::Fillet", "Fillet")
    fillet.Base = InputBox1
    __fillets__ = []
    __fillets__.append((1, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((5, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    InputBox1a = App.ActiveDocument.addObject("Part::Feature", "InputBox1a")
    InputBox1a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject("Fillet")
    doc.removeObject("InputBox1")

    doc.recompute()

    INPUT_LENGTH = SEAL_LENGTH + 2.5 + GROOVE_TOLERANCE
    INPUT_WIDTH = SEAL_WIDTH + 2.5 + GROOVE_TOLERANCE
    InputBox2 = doc.addObject("Part::Box", "InputBox2")
    InputBox2.Length = INPUT_LENGTH
    InputBox2.Width = INPUT_WIDTH
    InputBox2.Height = GROOVE_HEIGHT
    InputBox2.Placement = App.Placement(
        App.Vector(-(INPUT_LENGTH) / 2, -(INPUT_WIDTH) / 2, MAX_BASE_Y - GROOVE_HEIGHT),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    fillet = doc.addObject("Part::Fillet", "Fillet")
    fillet.Base = InputBox2
    __fillets__ = []
    __fillets__.append((1, 2.00, 2.00))
    __fillets__.append((3, 2.00, 2.00))
    __fillets__.append((5, 2.00, 2.00))
    __fillets__.append((7, 2.00, 2.00))
    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    InputBox2a = App.ActiveDocument.addObject("Part::Feature", "InputBox2a")
    InputBox2a.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject("Fillet")
    doc.removeObject("InputBox2")

    inputboxcut = doc.addObject("Part::Cut", "inputboxcut")
    inputboxcut.Base = InputBox2a
    inputboxcut.Tool = InputBox1a
    doc.recompute()

    inputbox = doc.addObject("Part::Feature", "inputbox")
    inputbox.Label = "inputbox"
    inputbox.Shape = Part.Solid(Part.Shell(inputboxcut.Shape.Faces))

    doc.removeObject("inputboxcut")
    doc.recompute()

    doc.removeObject("InputBox1a")
    doc.removeObject("InputBox2a")

    # cut out the hole

    basecut = doc.addObject("Part::Cut", "basecut")
    basecut.Base = baseholder
    basecut.Tool = inputbox
    doc.recompute()

    FullBase = doc.addObject("Part::Feature", "FullBase")
    FullBase.Shape = Part.Solid(Part.Shell(basecut.Shape.Faces))
    doc.recompute()

    doc.removeObject("basecut")

    doc.removeObject("inputbox")
    doc.removeObject(baseholder.Name)
    doc.recompute()
    return FullBase


def add_pcb_cutout(doc, baseholder):
    CutterBox = doc.addObject("Part::Box", "CutterBox")
    CutterBox.Length = CHIP_LENGTH
    CutterBox.Width = CHIP_WIDTH
    CutterBox.Height = CAVITY_HEIGHT
    CutterBox.Placement = App.Placement(
        App.Vector(
            -CHIP_LENGTH / 2, -CHIP_WIDTH / 2, FLOOR_THICKNESS - PLATE_THICKNESS
        ),
        App.Rotation(App.Vector(0, 0, 1), 0),
    )

    # TODO add the fillets for the corners and the inverted corner
    doc.recompute()
    cut = doc.addObject("Part::Cut", "cut")
    cut.Base = baseholder
    cut.Tool = CutterBox
    doc.recompute()

    newbaseholder = doc.addObject("Part::Feature", "newbaseholder")
    newbaseholder.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Name)
    doc.removeObject(baseholder.Name)
    doc.removeObject(CutterBox.Name)
    return newbaseholder


def make_screw_holes(doc, baseholder):
    ScrewCut1 = doc.addObject("Part::Cylinder", "ScrewCut1")
    ScrewCut1.Height = 100
    ScrewCut1.Radius = HEX_NUT_DIAM / 2

    ScrewCut1.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET,
            -50,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    ScrewCut2 = doc.addObject("Part::Cylinder", "ScrewCut2")
    ScrewCut2.Height = 100
    ScrewCut2.Radius = HEX_NUT_DIAM / 2
    ScrewCut2.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            -SCREW_LOCATION_L / 2 - FRONT_OFFSET,
            -50,
        ),
        App.Rotation(App.Vector(1, 0, 0), SCREW_ANGLE),
    )

    ScrewCut3 = doc.addObject("Part::Cylinder", "ScrewCut3")
    ScrewCut3.Height = 100
    ScrewCut3.Radius = HEX_NUT_DIAM / 2
    ScrewCut3.Placement = App.Placement(
        App.Vector(
            -SCREW_LOCATION_W / 2 + SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET,
            -50,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    ScrewCut4 = doc.addObject("Part::Cylinder", "ScrewCut4")
    ScrewCut4.Height = 100
    ScrewCut4.Radius = HEX_NUT_DIAM / 2
    ScrewCut4.Placement = App.Placement(
        App.Vector(
            +SCREW_LOCATION_W / 2 - SIDE_OFFSET,
            SCREW_LOCATION_L / 2 + FRONT_OFFSET,
            -50,
        ),
        App.Rotation(App.Vector(1, 0, 0), -SCREW_ANGLE),
    )

    cut = doc.addObject("Part::Cut", "Cut")
    cut.Base = baseholder
    cut.Tool = ScrewCut1
    doc.recompute()
    baseplate2pre0 = doc.addObject("Part::Feature", "baseplate2pre0")
    baseplate2pre0.Label = "baseplate2pre0"
    baseplate2pre0.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(baseholder.Label)

    cut = doc.addObject("Part::Cut", "Cut")
    cut.Base = baseplate2pre0
    cut.Tool = ScrewCut2
    doc.recompute()

    baseplate2pre1 = doc.addObject("Part::Feature", "baseplate2pre1")
    baseplate2pre1.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(baseplate2pre0.Label)

    cut = doc.addObject("Part::Cut", "Cut")
    cut.Base = baseplate2pre1
    cut.Tool = ScrewCut3
    doc.recompute()

    baseplate2pre2 = doc.addObject("Part::Feature", "baseplate2pre2")
    baseplate2pre2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(baseplate2pre1.Label)

    cut = doc.addObject("Part::Cut", "Cut")
    cut.Base = baseplate2pre2
    cut.Tool = ScrewCut4
    doc.recompute()

    baseplate2 = doc.addObject("Part::Feature", "baseplate2")
    baseplate2.Shape = Part.Solid(Part.Shell(cut.Shape.Faces))

    doc.removeObject(cut.Label)
    doc.removeObject(baseplate2pre2.Label)

    doc.removeObject(ScrewCut1.Label)
    doc.removeObject(ScrewCut2.Label)
    doc.removeObject(ScrewCut3.Label)
    doc.removeObject(ScrewCut4.Label)

    return baseplate2


def fillet_edges(doc, baseplate):
    fillet = doc.addObject("Part::Fillet", "Fillet")
    fillet.Base = baseplate

    __fillets__ = []

    # all these edges will likely change once the cutout box is rounded!
    __fillets__.append((52, 2.00, 2.00))
    __fillets__.append((53, 2.00, 2.00))
    __fillets__.append((96, 2.00, 2.00))
    __fillets__.append((97, 2.00, 2.00))

    __fillets__.append((122, 1.50, 1.50))
    __fillets__.append((124, 1.50, 1.50))
    __fillets__.append((131, 1.50, 1.50))
    __fillets__.append((139, 1.50, 1.50))

    __fillets__.append((40, 4.00, 4.00))
    __fillets__.append((9, 4.00, 4.00))

    fillet.Edges = __fillets__
    del __fillets__

    doc.recompute()

    baseplate2 = App.ActiveDocument.addObject("Part::Feature", "baseplate2")
    baseplate2.Label = "baseplate2"
    baseplate2.Shape = Part.Solid(Part.Shell(fillet.Shape.Faces))

    doc.removeObject(fillet.Name)
    doc.removeObject(baseplate.Name)

    doc.recompute()

    return baseplate2


def build_base_case(doc):
    curvedbase = create_curved_base(doc)
    baseholder = pcb_platform(doc, curvedbase)
    baseholder = add_surface(doc, baseholder)
    baseholder = add_groove(doc, baseholder)
    baseholder = add_pcb_cutout(doc, baseholder)
    baseholder = make_screw_holes(doc, baseholder)
    baseholder = fillet_edges(doc, baseholder)
