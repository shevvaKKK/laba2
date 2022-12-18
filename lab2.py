import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate

from StdReinfShapeBuilder.RotationAngles import RotationAngles
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties
from HandleService import HandleService


def check_allplan_version(build_ele, version):
    # Delete unused arguments
    del build_ele
    del version

    # Support all versions
    return True


def create_element(build_ele, doc):
    # Create Element
    element = CreateBridgeBeam(doc)

    return element.create(build_ele)


def modify_element_property(build_ele, name, value):
    if (name == "BeamHeight"):
        change = value - build_ele.TopShHeight.value - build_ele.RibHeight.value - build_ele.BotShUpHeight.value - build_ele.BotShLowHeight.value
        print(change)
        if (change < 0):
            change = abs(change)
            if (build_ele.TopShHeight.value > 330.):
                if (build_ele.TopShHeight.value - change < 330.):
                    change -= build_ele.TopShHeight.value - 330.
                    build_ele.TopShHeight.value = 330.
                else:
                    build_ele.TopShHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.BotShUpHeight.value > 170.):
                if (build_ele.BotShUpHeight.value - change < 170.):
                    change -= build_ele.BotShUpHeight.value - 170.
                    build_ele.BotShUpHeight.value = 170.
                else:
                    build_ele.BotShUpHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.BotShLowHeight.value > 160.):
                if (build_ele.BotShLowHeight.value - change < 160.):
                    change -= build_ele.BotShLowHeight.value - 160.
                    build_ele.BotShLowHeight.value = 160.
                else:
                    build_ele.BotShLowHeight.value -= change
                    change = 0.
            if (change != 0) and (build_ele.RibHeight.value > 475.):
                if (build_ele.RibHeight.value - change < 475.):
                    change -= build_ele.RibHeight.value - 475.
                    build_ele.RibHeight.value = 475.
                else:
                    build_ele.RibHeight.value -= change
                    change = 0.
        else:
            build_ele.RibHeight.value += change
        if (value - build_ele.TopShHeight.value - 45.5 < build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = value - build_ele.TopShHeight.value - 45.5   
    elif (name == "TopShHeight"):
        build_ele.BeamHeight.value = value + build_ele.RibHeight.value + build_ele.BotShUpHeight.value + build_ele.BotShLowHeight.value
    elif (name == "RibHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.BotShUpHeight.value + build_ele.BotShLowHeight.value
    elif (name == "BotShUpHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.RibHeight.value + build_ele.BotShLowHeight.value
        if (value + build_ele.BotShLowHeight.value + 45.5 > build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = value + build_ele.BotShLowHeight.value + 45.5
    elif (name == "BotShLowHeight"):
        build_ele.BeamHeight.value = value + build_ele.TopShHeight.value + build_ele.RibHeight.value + build_ele.BotShUpHeight.value
        if (build_ele.BotShUpHeight.value + value + 45.5 > build_ele.HoleHeight.value):
            build_ele.HoleHeight.value = build_ele.BotShUpHeight.value + value + 45.5
    elif (name == "HoleHeight"):
        if (value > build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5):
            build_ele.HoleHeight.value = build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5
        elif (value < build_ele.BotShLowHeight.value + build_ele.BotShUpHeight.value + 45.5):
            build_ele.HoleHeight.value = build_ele.BotShLowHeight.value + build_ele.BotShUpHeight.value + 45.5
    elif (name == "HoleDepth"):
        if (value >= build_ele.BeamLength.value / 2.):
            build_ele.HoleDepth.value = build_ele.BeamLength.value / 2. - 45.5

    return True


def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)

    if (handle_prop.handle_id == "BeamHeight"):
        build_ele.RibHeight.value = build_ele.BeamHeight.value - build_ele.TopShHeight.value - build_ele.BotShLowHeight.value - build_ele.BotShUpHeight.value
        if (build_ele.HoleHeight.value > build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5):
            build_ele.HoleHeight.value = build_ele.BeamHeight.value - build_ele.TopShHeight.value - 45.5

    return create_element(build_ele, doc)

class CreateBridgeBeam():

    def __init__(self, doc):

        self.model_ele_list = []
        self.handle_list = []
        self.document = doc
        
    def create(self, build_ele):
        
        self._top_sh_width = build_ele.TopShWidth.value
        self._top_sh_height = build_ele.TopShHeight.value

        self._bot_sh_width = build_ele.BotShWidth.value
        self._bot_sh_up_height = build_ele.BotShUpHeight.value
        self._bot_sh_low_height = build_ele.BotShLowHeight.value
        self._bot_sh_height = self._bot_sh_up_height + self._bot_sh_low_height

        if (build_ele.RibThick.value > min(self._top_sh_width, self._bot_sh_width)):
            build_ele.RibThick.value = min(self._top_sh_width, self._bot_sh_width)        
        self._rib_thickness = build_ele.RibThick.value
        self._rib_height = build_ele.RibHeight.value

        self._beam_length = build_ele.BeamLength.value
        self._beam_width = max(self._top_sh_width, self._bot_sh_width)
        self._beam_height = build_ele.BeamHeight.value

        self._hole_depth = build_ele.HoleDepth.value
        self._hole_height = build_ele.HoleHeight.value

        self._angleX = build_ele.RotationAngleX.value
        self._angleY = build_ele.RotationAngleY.value
        self._angleZ = build_ele.RotationAngleZ.value

        self.create_beam(build_ele)
        self.create_handles(build_ele)
        
        AllplanBaseElements.ElementTransform(AllplanGeo.Vector3D(), self._angleX, self._angleY, self._angleZ, self.model_ele_list)

        rot_angles = RotationAngles(self._angleX, self._angleY, self._angleZ)
        HandleService.transform_handles(self.handle_list, rot_angles.get_rotation_matrix())
        
        return (self.model_ele_list, self.handle_list)


    def create_beam(self, build_ele):
        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = build_ele.Color3.value
        com_prop.Stroke = 1

        # bottom shelf
        bottom_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0., 0.), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._bot_sh_width, self._beam_length, self._bot_sh_height)

        edges = AllplanUtil.VecSizeTList()
        edges.append(10)
        edges.append(8)
        err, bottom_shelf = AllplanGeo.ChamferCalculus.Calculate(bottom_shelf, edges, 20., False)
        
        # top shelf
        top_shelf = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - self._top_sh_height), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._top_sh_width, self._beam_length, self._top_sh_height)

        top_shelf_notch = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0., self._beam_height - 45.), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), 60., self._beam_length, 45.)
        err, top_shelf = AllplanGeo.MakeSubtraction(top_shelf, top_shelf_notch)
        if not GeometryValidate.polyhedron(err):
            return
        top_shelf_notch = AllplanGeo.Move(top_shelf_notch, AllplanGeo.Vector3D(self._top_sh_width - 60., 0, 0))
        err, top_shelf = AllplanGeo.MakeSubtraction(top_shelf, top_shelf_notch)
        if not GeometryValidate.polyhedron(err):
            return
        
        err, beam = AllplanGeo.MakeUnion(bottom_shelf, top_shelf)
        if not GeometryValidate.polyhedron(err):
            return

        # rib
        rib = AllplanGeo.BRep3D.CreateCuboid(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0., 0., self._bot_sh_height), AllplanGeo.Vector3D(1, 0, 0), AllplanGeo.Vector3D(0, 0, 1)), self._beam_width, self._beam_length, self._rib_height)
        
        err, beam = AllplanGeo.MakeUnion(beam, rib)
        if not GeometryValidate.polyhedron(err):
            return
        
        # left and right notches
        left_notch_pol = AllplanGeo.Polygon2D()
        left_notch_pol += AllplanGeo.Point2D((self._beam_width - self._rib_thickness) / 2., self._beam_height - self._top_sh_height)
        left_notch_pol += AllplanGeo.Point2D((self._beam_width - self._rib_thickness) / 2., self._bot_sh_height)
        left_notch_pol += AllplanGeo.Point2D((self._beam_width - self._bot_sh_width) / 2., self._bot_sh_low_height)
        left_notch_pol += AllplanGeo.Point2D(0., self._bot_sh_low_height)     
        left_notch_pol += AllplanGeo.Point2D(0., self._beam_height - 100.)
        left_notch_pol += AllplanGeo.Point2D(0., self._beam_height - 100.)
        left_notch_pol += AllplanGeo.Point2D((self._beam_width - self._top_sh_width) / 2., self._beam_height - 100.)
        left_notch_pol += AllplanGeo.Point2D((self._beam_width - self._rib_thickness) / 2., self._beam_height - self._top_sh_height)
        if not GeometryValidate.is_valid(left_notch_pol):
            return
        
        path = AllplanGeo.Polyline3D()
        path += AllplanGeo.Point3D(0, 0, 0)
        path += AllplanGeo.Point3D(0, build_ele.BeamLength.value, 0)

        err, notches = AllplanGeo.CreatePolyhedron(left_notch_pol, AllplanGeo.Point2D(0., 0.), path)
        if GeometryValidate.polyhedron(err):
            edges = AllplanUtil.VecSizeTList()
            if (self._rib_thickness == self._bot_sh_width):
                edges.append(0)
            elif (self._rib_thickness == self._top_sh_width):
                edges.append(1)
            else:
                edges.append(0)
                edges.append(2)
            err, notches = AllplanGeo.FilletCalculus3D.Calculate(notches, edges, 100., False)

            plane = AllplanGeo.Plane3D(AllplanGeo.Point3D(self._beam_width / 2., 0, 0), AllplanGeo.Vector3D(1, 0, 0))
            right_notch = AllplanGeo.Mirror(notches, plane)

            err, notches = AllplanGeo.MakeUnion(notches, right_notch)
            if not GeometryValidate.polyhedron(err):
                return
            
            err, beam = AllplanGeo.MakeSubtraction(beam, notches)
            if not GeometryValidate.polyhedron(err):
                return

        sling_holes = AllplanGeo.BRep3D.CreateCylinder(AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0,build_ele.HoleDepth.value, build_ele.HoleHeight.value), AllplanGeo.Vector3D(0, 0, 1), AllplanGeo.Vector3D(1, 0, 0)), 45.5, self._beam_width)
        
        sling_hole_moved = AllplanGeo.Move(sling_holes, AllplanGeo.Vector3D(0., self._beam_length - self._hole_depth * 2, 0))

        err, sling_holes = AllplanGeo.MakeUnion(sling_holes, sling_hole_moved)
        if not GeometryValidate.polyhedron(err):
            return
            
        err, beam = AllplanGeo.MakeSubtraction(beam, sling_holes)
        if not GeometryValidate.polyhedron(err):
            return

        self.model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop, beam))
        

    def create_handles(self, build_ele):
        handle1 = HandleProperties("BeamLength",
                                   AllplanGeo.Point3D(0., self._beam_length, 0.),
                                   AllplanGeo.Point3D(0, 0, 0),
                                   [("BeamLength", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle1)

        handle2 = HandleProperties("BeamHeight",
                                   AllplanGeo.Point3D(0., 0., self._beam_height),
                                   AllplanGeo.Point3D(0, 0, 0),
                                   [("BeamHeight", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle2)
        
        handle3 = HandleProperties("TopShWidth",
                                   AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2. + self._top_sh_width, 0., self._beam_height - 45.),
                                   AllplanGeo.Point3D((self._beam_width - self._top_sh_width) / 2., 0, self._beam_height - 45.),
                                   [("TopShWidth", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle3)

        handle4 = HandleProperties("BotShWidth",
                                   AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2. + self._bot_sh_width, 0., self._bot_sh_low_height),
                                   AllplanGeo.Point3D((self._beam_width - self._bot_sh_width) / 2., 0, self._bot_sh_low_height),
                                   [("BotShWidth", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle4)
        
        handle5 = HandleProperties("RibThick",
                                   AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2. + self._rib_thickness, 0., self._beam_height / 2.),
                                   AllplanGeo.Point3D((self._beam_width - self._rib_thickness) / 2., 0, self._beam_height / 2.),
                                   [("RibThick", HandleDirection.point_dir)],
                                   HandleDirection.point_dir, True)
        self.handle_list.append(handle5)