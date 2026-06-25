import os.path, sys
from typing import Dict, List, Sequence, Tuple, Optional
from xml.etree.ElementTree import Element, ElementTree, SubElement
import xml.etree.ElementTree as ET
import adsk, adsk.core, adsk.fusion

from .parser import Configurator
from .parts import Joint, Link
from . import utils
from shutil import copytree


def visible_to_stl(
    design: adsk.fusion.Design,
    save_dir: str,
    root: adsk.fusion.Component,
    accuracy: adsk.fusion.MeshRefinementSettings,
    sub_mesh: bool,
    body_mapper: Dict[str, List[Tuple[adsk.fusion.BRepBody, str]]],
    _app,
):
    """
    export top-level components as a single stl file into "save_dir/"

    Parameters
    ----------
    design: adsk.fusion.Design
        fusion design document
    save_dir: str
        directory path to save
    root: adsk.fusion.Component
        root component of the design
    accuracy: adsk.fusion.MeshRefinementSettings enum
        accuracy value to use for stl export
    component_map: list
        list of all bodies to use for stl export
    """

    # create a single exportManager instance
    exporter = design.exportManager

    newDoc: adsk.core.Document = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType, True)
    newDes = newDoc.products.itemByProductType("DesignProductType")
    assert isinstance(newDes, adsk.fusion.Design)
    newRoot = newDes.rootComponent

    # get the script location
    save_dir = os.path.join(save_dir, "meshes")
    try:
        os.mkdir(save_dir)
    except:
        pass

    try:
        for name, bodies in body_mapper.items():
            if not bodies:
                continue

            # Create a new exporter in case its a memory thing
            exporter = design.exportManager

            occName = utils.format_name(name)
            stl_exporter(exporter, accuracy, newRoot, [b for b, _ in bodies], os.path.join(save_dir, occName))

            if sub_mesh and len(bodies) > 1:
                for body, body_name in bodies:
                    if body.isVisible:
                        stl_exporter(exporter, accuracy, newRoot, [body], os.path.join(save_dir, body_name))
    finally:
        newDoc.close(False)


def stl_exporter(exportMgr, accuracy, newRoot, body_lst, filename):
    """Copy a component to a new document, save, then delete.

    Modified from solution proposed by BrianEkins https://EkinsSolutions.com

    Parameters
    ----------
    exportMgr : _type_
        _description_
    newRoot : _type_
        _description_
    body_lst : _type_
        _description_
    filename : _type_
        _description_
    """

    tBrep = adsk.fusion.TemporaryBRepManager.get()

    bf = newRoot.features.baseFeatures.add()
    bf.startEdit()

    for body in body_lst:
        tBody = tBrep.copy(body)
        newRoot.bRepBodies.add(tBody, bf)

    bf.finishEdit()
    stlOptions = exportMgr.createSTLExportOptions(newRoot, f"{filename}.stl")
    stlOptions.meshRefinement = accuracy
    exportMgr.execute(stlOptions)

    bf.deleteMe()


class Writer:

    def __init__(self, save_dir: str, config: Configurator, export_format: Optional[str] = None) -> None:
        self.save_dir = save_dir
        self.config = config
        self.export_format = export_format  # 'URDF' or 'Xacro'

    def write_urdf(self) -> None:
        """Write each component of the xml structure to file, in URDF or Xacro format."""
        try:
            os.mkdir(self.save_dir)
        except:
            pass

        if self.export_format == 'URDF':
            self._write_urdf_plain()
        else:  # default to Xacro
            self._write_xacro()

    # ============================================================
    # URDF (plain) – materials embedded directly in the main file
    # ============================================================
    def _write_urdf_plain(self):
        """Write a plain URDF file with materials defined inside <robot>."""
        file_name = os.path.join(self.save_dir, f"{self.config.name}.urdf")

        # Select links/joints (with or without extra_links)
        if self.config.extra_links:
            links = self.config.links.copy()
            for link in self.config.extra_links:
                del links[link]
            joints = {
                joint: self.config.joints[joint]
                for joint in self.config.joints
                if self.config.joints[joint].child not in self.config.extra_links
            }
        else:
            links = self.config.links
            joints = self.config.joints

        # Write the main URDF (with embedded materials)
        self._write_urdf_xml(file_name, links, joints, embed_materials=True)

        # No separate materials.urdf needed
        # (we skip _write_materials_urdf)

    def _write_urdf_xml(self, file_name: str, links: Dict[str, Link], joints: Dict[str, Joint], embed_materials: bool = True):
        """Write plain URDF XML. If embed_materials=True, materials are defined inside <robot>."""
        robot = Element("robot", {"name": self.config.name})

        # ---- Material embedding ----
        if embed_materials:
            for color_name, rgba in self.config.color_dict.items():
                mat = SubElement(robot, "material", {"name": color_name})
                SubElement(mat, "color", {"rgba": rgba})

        # Add dummy link
        SubElement(robot, "link", {"name": "dummy_link"})
        assert self.config.base_link is not None
        dummy_joint = SubElement(robot, "joint", {"name": "dummy_link_joint", "type": "fixed"})
        SubElement(dummy_joint, "parent", {"link": "dummy_link"})
        SubElement(dummy_joint, "child", {"link": self.config.base_link_name})

        # Add links and joints
        for _, link in links.items():
            xml = link.link_xml()
            if xml is not None:
                robot.append(xml)
        for _, joint in joints.items():
            robot.append(joint.joint_xml())

        tree = ElementTree(robot)
        ET.indent(tree, space="   ")
        with open(file_name, mode="wb") as f:
            tree.write(f, "utf-8", xml_declaration=True)
            f.write(b"\n")

    # ============================================================
    # XACRO – separate materials file with xacro:include
    # ============================================================
    def _write_xacro(self):
        """Original xacro-based write method."""
        file_name = os.path.join(self.save_dir, f"{self.config.name}.xacro")
        if self.config.extra_links:
            links = self.config.links.copy()
            for link in self.config.extra_links:
                del links[link]
            joints = {
                joint: self.config.joints[joint]
                for joint in self.config.joints
                if self.config.joints[joint].child not in self.config.extra_links
            }
            self._write_xacro_xml(file_name, links, joints)
            file_name_full = os.path.join(self.save_dir, f"{self.config.name}-full.xacro")
            self._write_xacro_xml(file_name_full, {link: self.config.links[link] for link in self.config.extra_links}, {})
        else:
            self._write_xacro_xml(file_name, self.config.links, self.config.joints)

        # Materials as separate file (with xacro namespace)
        material_file_name = os.path.join(self.save_dir, f"materials.xacro")
        self._write_materials_xacro(material_file_name)

    def _write_materials_xacro(self, material_file_name):
        robot = Element("robot", {"name": self.config.name, "xmlns:xacro": "http://www.ros.org/wiki/xacro"})
        for color_name, rgba in self.config.color_dict.items():
            material = SubElement(robot, "material", {"name": color_name})
            SubElement(material, "color", {"rgba": rgba})
        tree = ElementTree(robot)
        ET.indent(tree, space="   ")
        with open(material_file_name, mode="wb") as f:
            tree.write(f, "utf-8", xml_declaration=True)
            f.write(b"\n")

    def _write_xacro_xml(self, file_name: str, links: Dict[str, Link], joints: Dict[str, Joint]):
        robot = Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro", "name": self.config.name})
        SubElement(robot, "xacro:include", {"filename": f"$(find {self.config.name})/urdf/materials.xacro"})

        # Add dummy link
        SubElement(robot, "link", {"name": "dummy_link"})
        assert self.config.base_link is not None
        dummy_joint = SubElement(robot, "joint", {"name": "dummy_link_joint", "type": "fixed"})
        SubElement(dummy_joint, "parent", {"link": "dummy_link"})
        SubElement(dummy_joint, "child", {"link": self.config.base_link_name})

        for _, link in links.items():
            xml = link.link_xml()
            if xml is not None:
                robot.append(xml)
        for _, joint in joints.items():
            robot.append(joint.joint_xml())

        tree = ElementTree(robot)
        ET.indent(tree, space="   ")
        with open(file_name, mode="wb") as f:
            tree.write(f, "utf-8", xml_declaration=True)
            f.write(b"\n")


# =============================================================================
# Helper functions (pyBullet, ROS2, Gazebo, MoveIt)
# =============================================================================

def write_hello_pybullet(robot_name, save_dir) -> None:
    """Writes a sample script which loads the URDF in pybullet

    Modified from https://github.com/yanshil/Fusion2PyBullet

    Parameters
    ----------
    robot_name : str
        name to use for directory
    save_dir : str
        path to store file
    """

    robot_urdf = f"{robot_name}.urdf"  ## basename of robot.urdf
    file_name = os.path.join(save_dir, "hello_bullet.py")
    hello_pybullet = """
import pybullet as p
import os
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
dir = os.path.abspath(os.path.dirname(__file__))
robot_urdf = "TEMPLATE.urdf"
dir = os.path.join(dir,'urdf')
robot_urdf=os.path.join(dir,robot_urdf)
robotId = p.loadURDF(robot_urdf,cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()
"""
    hello_pybullet = hello_pybullet.replace("TEMPLATE.urdf", robot_urdf)
    with open(file_name, mode="w") as f:
        f.write(hello_pybullet)
        f.write("\n")


def copy_ros2(save_dir, package_name) -> None:
    # Use current directory to find `package_ros2`
    package_ros2_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/package_ros2/"
    copy_package(save_dir, package_ros2_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/robot_description.launch.py", package_name)


def copy_gazebo(save_dir, package_name) -> None:
    # Use current directory to find `gazebo_package`
    gazebo_package_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/gazebo_package/"
    copy_package(save_dir, gazebo_package_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/robot_description.launch.py", package_name)  # Also include rviz alone
    update_package_name(save_dir + "/launch/gazebo.launch.py", package_name)


def copy_moveit(save_dir, package_name) -> None:
    # Use current directory to find `moveit_package`
    moveit_package_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/moveit_package/"
    copy_package(save_dir, moveit_package_path)
    update_cmakelists(save_dir, package_name)
    update_package_xml(save_dir, package_name)
    update_package_name(save_dir + "/launch/setup_assistant.launch.py", package_name)


def copy_package(save_dir, package_dir) -> None:
    try:
        os.mkdir(save_dir + "/launch")
    except:
        pass
    try:
        os.mkdir(save_dir + "/urdf")
    except:
        pass
    copytree(package_dir, save_dir, dirs_exist_ok=True)


# =============================================================================
# Safe file update functions (without fileinput)
# =============================================================================

def update_cmakelists(save_dir, package_name) -> None:
    """Replaces the project name in CMakeLists.txt in a safe way."""
    file_name = save_dir + "/CMakeLists.txt"
    try:
        with open(file_name, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        return  # if the file doesn't exist, just ignore

    with open(file_name, 'w', encoding='utf-8') as f:
        for line in lines:
            if "project(fusion2urdf)" in line:
                f.write("project(" + package_name + ")\n")
            else:
                f.write(line)


def update_package_name(file_name, package_name) -> None:
    """Replaces 'fusion2urdf' with the package name in a file."""
    try:
        with open(file_name, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        return

    with open(file_name, 'w', encoding='utf-8') as f:
        for line in lines:
            if "fusion2urdf" in line:
                f.write(line.replace("fusion2urdf", package_name))
            else:
                f.write(line)


def update_package_xml(save_dir, package_name) -> None:
    """Updates package.xml with the package name."""
    file_name = save_dir + "/package.xml"
    try:
        with open(file_name, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except FileNotFoundError:
        return

    with open(file_name, 'w', encoding='utf-8') as f:
        for line in lines:
            if "<name>" in line:
                f.write("  <name>" + package_name + "</name>\n")
            elif "<description>" in line:
                f.write("<description>The " + package_name + " package</description>\n")
            else:
                f.write(line)