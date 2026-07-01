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

    exporter = design.exportManager

    newDoc: adsk.core.Document = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType, True)
    newDes = newDoc.products.itemByProductType("DesignProductType")
    assert isinstance(newDes, adsk.fusion.Design)
    newRoot = newDes.rootComponent

    save_dir = os.path.join(save_dir, "meshes")
    try:
        os.makedirs(save_dir, exist_ok=True)
    except:
        pass

    try:
        for name, bodies in body_mapper.items():
            if not bodies:
                continue

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

    def __init__(self, save_dir: str, config: Configurator, export_format: Optional[str] = None, ros_version: Optional[str] = "ROS 2", robot_name: Optional[str] = None) -> None:
        self.save_dir = save_dir
        self.config = config
        self.export_format = export_format  # 'URDF' or 'Xacro'
        self.ros_version = ros_version      # 'ROS 1' or 'ROS 2'
        self.robot_name = robot_name        # name of the robot (for package name)

        if not self.robot_name:
            self.robot_name = config.name

    def write_urdf(self) -> None:
        """Write each component of the xml structure to file, in URDF or Xacro format."""
        try:
            os.makedirs(self.save_dir, exist_ok=True)
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

        self._write_urdf_xml(file_name, links, joints, embed_materials=True)

    def _write_urdf_xml(self, file_name: str, links: Dict[str, Link], joints: Dict[str, Joint], embed_materials: bool = True):
        """Write plain URDF XML. If embed_materials=True, materials are defined inside <robot>."""
        robot = Element("robot", {"name": self.config.name})

        if embed_materials:
            for color_name, rgba in self.config.color_dict.items():
                mat = SubElement(robot, "material", {"name": color_name})
                SubElement(mat, "color", {"rgba": rgba})

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

    # ============================================================
    # XACRO – separate materials file with xacro:include
    # ============================================================
    def _write_xacro(self):
        """Write Xacro files (main + materials)."""
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
        robot_name = self.robot_name if self.robot_name else self.config.name
        SubElement(robot, "xacro:include", {"filename": f"$(find {robot_name})/urdf/materials.xacro"})

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
# Helpers (pyBullet, ROS1/ROS2 templates)
# =============================================================================

def write_hello_pybullet(robot_name, save_dir) -> None:
    """Writes a sample script which loads the URDF in pybullet"""
    robot_urdf = f"{robot_name}.urdf"
    file_name = os.path.join(save_dir, "hello_bullet.py")
    hello_pybullet = """
import pybullet as p
import os
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
dir = os.path.abspath(os.path.dirname(__file__))
robot_urdf = "TEMPLATE.urdf"
dir = os.path.join(dir,'urdf')
robot_urdf=os.path.join(dir,robot_urdf)
robotId = p.loadURDF(robot_urdf,cubeStartPos, cubeStartOrientation, 
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


# -----------------------------------------------------------------------------
# ΝΕΑ: Ενιαία συνάρτηση αντιγραφής template
# -----------------------------------------------------------------------------
def copy_template(save_dir: str, robot_name: str, ros_version: str, target_platform: str) -> None:
    """
    Αντιγράφει το κατάλληλο template από το φάκελο templates/ros{1,2}/{target_platform}/
    και αντικαθιστά το placeholder %ROBOT_NAME% σε όλα τα αρχεία.

    Parameters
    ----------
    save_dir : str
        φάκελος προορισμού (π.χ. Harper_Final_description/)
    robot_name : str
        όνομα του ρομπότ (αντικαθιστά το %ROBOT_NAME%)
    ros_version : str
        'ROS 1' ή 'ROS 2'
    target_platform : str
        'rviz', 'Gazebo', 'MoveIt' (case-sensitive)
    """
    # Προσδιορισμός του φακέλου ROS
    ros_folder = "ros1" if ros_version == "ROS 1" else "ros2"
    # Χαρτογράφηση του target_platform στο όνομα του υποφακέλου (προσοχή σε κεφαλαία)
    platform_map = {
        "rviz": "rviz",
        "Gazebo": "gazebo",
        "MoveIt": "moveit",
    }
    platform_folder = platform_map.get(target_platform)
    if not platform_folder:
        utils.log(f"WARNING: Unknown target platform '{target_platform}'. Skipping template copy.")
        return

    # Βάση του φακέλου templates
    template_base = os.path.dirname(os.path.abspath(os.path.dirname(__file__))) + "/templates/"
    template_dir = os.path.join(template_base, ros_folder, platform_folder)

    # Έλεγχος ύπαρξης του φακέλου
    if not os.path.exists(template_dir):
        utils.log(f"ERROR: Template folder '{template_dir}' does not exist. Skipping.")
        return

    # Αντιγραφή του template
    copy_package(save_dir, template_dir)

    # Ενημέρωση όλων των αρχείων με το %ROBOT_NAME%
    # 1. CMakeLists.txt και package.xml
    update_file(save_dir + "/CMakeLists.txt", robot_name)
    update_file(save_dir + "/package.xml", robot_name)

    # 2. Όλα τα αρχεία στον φάκελο launch/
    launch_dir = os.path.join(save_dir, "launch")
    if os.path.exists(launch_dir):
        for root, dirs, files in os.walk(launch_dir):
            for file in files:
                file_path = os.path.join(root, file)
                update_file(file_path, robot_name)

    # 3. (Προαιρετικά) το urdf.rviz (δεν περιέχει %ROBOT_NAME% συνήθως, αλλά το κάνουμε για σιγουριά)
    rviz_file = os.path.join(launch_dir, "urdf.rviz")
    if os.path.exists(rviz_file):
        update_file(rviz_file, robot_name)


def copy_package(save_dir: str, package_dir: str) -> None:
    """Copy a package template directory."""
    try:
        os.makedirs(save_dir + "/launch", exist_ok=True)
    except:
        pass
    try:
        os.makedirs(save_dir + "/urdf", exist_ok=True)
    except:
        pass
    copytree(package_dir, save_dir, dirs_exist_ok=True)


def update_file(file_path: str, robot_name: str) -> None:
    """
    Βοηθητική συνάρτηση που αντικαθιστά όλες τις εμφανίσεις του '%ROBOT_NAME%'
    στο περιεχόμενο ενός αρχείου με το όνομα του ρομπότ.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        return

    content = content.replace("%ROBOT_NAME%", robot_name)

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)