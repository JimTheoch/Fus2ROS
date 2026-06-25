'''
module to parse fusion file 
'''

from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple, Union
from dataclasses import dataclass, field

import adsk.core, adsk.fusion
import numpy as np
from . import transforms
from . import parts
from . import utils
from collections import OrderedDict, defaultdict

@dataclass(frozen=True, kw_only=True, eq=False)
class JointInfo:
    name: str
    parent: str
    child: str
    type: str = "fixed"
    origin: adsk.core.Vector3D = field(default_factory=adsk.core.Vector3D.create)
    axis: adsk.core.Vector3D = field(default_factory=adsk.core.Vector3D.create)
    upper_limit: float = 0.0
    lower_limit: float = 0.0
    continuous: bool = False

class Hierarchy:
    total_components = 0

    def __init__(self, component) -> None:
        self.children: List["Hierarchy"] = []
        self.component: adsk.fusion.Occurrence = component
        self.name: str = component.name
        Hierarchy.total_components += 1
        if utils.LOG_DEBUG:
            utils.log(f"... {Hierarchy.total_components}. Collected {self.name}...")

    def _add_child(self, c: "Hierarchy") -> None:
        self.children.append(c)

    def get_children(self) -> List["Hierarchy"]:
        return self.children        

    def get_all_children(self) -> Dict[str, "Hierarchy"]:
        child_map = OrderedDict()
        parent_stack: List["Hierarchy"] = []
        parent_stack += self.get_children()
        while parent_stack:
            tmp = parent_stack.pop(0)
            child_map[tmp.component.entityToken] = tmp 
            parent_stack += tmp.get_children()
        return child_map

    def get_flat_body(self) -> List[adsk.fusion.BRepBody]:
        child_list = []
        body_list: List[List[adsk.fusion.BRepBody]] = []

        child_set = list(self.get_all_children().values())

        if len(child_set) == 0:
            body_list.append(list(self.component.bRepBodies))

        child_list = [x.children for x in child_set if len(x.children)>0]
        parent_stack : List[Hierarchy] = []
        for c in child_list:
            for _c in c:
                parent_stack.append(_c)

        closed_set = set()

        while len(parent_stack) != 0:
            tmp = parent_stack.pop()
            closed_set.add(tmp)
            if tmp.component.bRepBodies.count > 0:
                body_list.append(list(tmp.component.bRepBodies))

            if len(tmp.children)> 0:
                child_set = list(self.get_all_children().values())

                child_list = [x.children for x in child_set if len(x.children)>0]
                for c in child_list:
                    for _c in c:
                        if _c not in closed_set:
                            parent_stack.append(_c)

        flat_bodies: List[adsk.fusion.BRepBody] = []
        for body in body_list:
            flat_bodies.extend(body)

        return flat_bodies

    @staticmethod
    def traverse(occurrences: adsk.fusion.OccurrenceList, parent: Optional["Hierarchy"] = None) -> "Hierarchy":
        assert occurrences
        for i in range(0, occurrences.count):
            occ = occurrences.item(i)

            cur = Hierarchy(occ)

            if parent is None: 
                pass
            else: 
                parent._add_child(cur)

            if occ.childOccurrences:
                Hierarchy.traverse(occ.childOccurrences, parent=cur)
        return cur

def get_origin(o: Optional[adsk.core.Base]) -> Union[adsk.core.Vector3D, None]:
    if isinstance(o, adsk.fusion.JointGeometry):
        return o.origin.asVector()
    elif o is None:
        return None
    elif isinstance(o, adsk.fusion.JointOrigin):
        return get_origin(o.geometry)
    else:
        utils.fatal(f"parser.get_origin: unexpected {o} of type {type(o)}")
    
def get_context_name(c: Optional[adsk.fusion.Occurrence]) -> str:
    return c.name if c is not None else 'ROOT level'

def getMatrixFromRoot(occ: Optional[adsk.fusion.Occurrence]) -> adsk.core.Matrix3D:
    mat = adsk.core.Matrix3D.create()
    while occ is not None:
        mat.transformBy(occ.transform2)
        occ = occ.assemblyContext
    return mat

class Configurator:

    joint_types: Dict[adsk.fusion.JointTypes, str] = {
        adsk.fusion.JointTypes.RigidJointType: "fixed",
        adsk.fusion.JointTypes.RevoluteJointType: "revolute",
        adsk.fusion.JointTypes.SliderJointType: "prismatic",
        adsk.fusion.JointTypes.CylindricalJointType: "Cylindrical_unsupported",
        adsk.fusion.JointTypes.PinSlotJointType: "PinSlot_unsupported",
        adsk.fusion.JointTypes.PlanarJointType: "planar",
        adsk.fusion.JointTypes.BallJointType: "Ball_unsupported",
    }

    def __init__(self, root: adsk.fusion.Component, scale: float, cm: float, name: str, name_map: Dict[str, str], merge_links: Dict[str, List[str]], locations: Dict[str, Dict[str, str]], extra_links: Sequence[str], root_name: Optional[str]) -> None:
        self.root = root
        self.occ = root.occurrences.asList
        self.inertia_accuracy = adsk.fusion.CalculationAccuracy.LowCalculationAccuracy

        self.sub_mesh = False
        self.links_by_token: Dict[str, str] = OrderedDict()
        self.links_by_name : Dict[str, adsk.fusion.Occurrence] = OrderedDict()
        self.joints_dict: Dict[str, JointInfo] = OrderedDict()
        self.body_dict: Dict[str, List[Tuple[adsk.fusion.BRepBody, str]]] = OrderedDict()
        self.material_dict: Dict[str, str] = OrderedDict()
        self.color_dict: Dict[str, str] = OrderedDict()
        self.links: Dict[str, parts.Link] = OrderedDict()
        self.joints: Dict[str, parts.Joint] = OrderedDict()
        self.locs: Dict[str, List[parts.Location]] = OrderedDict()
        self.scale = scale
        self.cm = cm
        parts.Link.scale = str(self.scale)
        self.eps = 1e-7 / max(self.scale, 1e-6)
        self.base_link: Optional[adsk.fusion.Occurrence] = None
        self.component_map: Dict[str, Hierarchy] = OrderedDict()
        self.bodies_collected: Set[str] = set()
        self.name_map = name_map
        self.merge_links = merge_links
        self.locations = locations
        self.extra_links = extra_links

        self.root_node: Optional[Hierarchy] = None
        self.root_name = root_name

        self.name = name
        self.mesh_folder = f'{name}/meshes/'

    def close_enough(self, a, b) -> bool:
        if isinstance(a, float) and isinstance(b, float):
            return abs(a-b) < self.eps
        elif isinstance(a, list) and isinstance(b, list):
            assert len(a) == len(b)
            return all((self.close_enough(aa,bb) for aa,bb in zip(a,b)))
        elif isinstance(a, tuple) and isinstance(b, tuple):
            assert len(a) == len(b)
            return all((self.close_enough(aa,bb) for aa,bb in zip(a,b)))
        elif isinstance(a, adsk.core.Vector3D) and isinstance(b, adsk.core.Vector3D):
            return self.close_enough(a.asArray(), b.asArray())
        elif isinstance(a, adsk.core.Point3D) and isinstance(b, adsk.core.Point3D):
            return self.close_enough(a.asArray(), b.asArray())
        elif isinstance(a, adsk.core.Matrix3D) and isinstance(b, adsk.core.Matrix3D):
            return self.close_enough(a.asArray(), b.asArray())
        else:
            utils.fatal(f"parser.Configurator.close_enough: {type(a)} and {type(b)}: not supported")
        
    def get_scene_configuration(self):
        Hierarchy.total_components = 0
        utils.log("* Traversing the hierarchy *")
        self.root_node = Hierarchy(self.root)
        occ_list=self.root.occurrences.asList
        Hierarchy.traverse(occ_list, self.root_node)
        utils.log(f"* Collected {Hierarchy.total_components} components, processing *")
        self.component_map = self.root_node.get_all_children()
        utils.log("* Processing sub-bodies *")
        self.get_sub_bodies()

        return self.component_map

    def get_sub_bodies(self) -> None:
        self.body_mapper: Dict[str, List[adsk.fusion.BRepBody]] = defaultdict(list)

        assert self.root_node is not None

        for v in self.root_node.children:
            
            children = set()
            children.update(v.children)

            top_level_body = [x for x in v.component.bRepBodies if x.isVisible]
            
            if top_level_body != []:
                self.body_mapper[v.component.entityToken].extend(top_level_body)

            while children:
                cur = children.pop()
                children.update(cur.children)
                sub_level_body = [x for x in cur.component.bRepBodies if x.isVisible]
                
                self.body_mapper[cur.component.entityToken].extend(sub_level_body)

    def get_joint_preview(self) -> Dict[str, JointInfo]:
        self._joints()
        return self.joints_dict

    def parse(self):
        self._base()
        self._joints()
        self._links()
        self._materials()
        self._build()

    def _base(self):
        if self.root_name:
            for oc in self.root.allOccurrences:
                if oc.name == self.root_name:
                    self.base_link = oc
                    utils.log(f"Using user-selected root from dropdown: {self.base_link.name}")
                    break
            
            if self.base_link is None:
                for oc in self.root.allOccurrences:
                    if self.root_name in oc.name:
                        self.base_link = oc
                        utils.log(f"Using user-selected root (partial match): {self.base_link.name}")
                        break
            
            if self.base_link is None:
                utils.fatal(f"Could not find component '{self.root_name}' in the model. Please refresh the component list.")
            return
        
        for oc in self.root.allOccurrences:
            if oc.isGrounded:
                self.base_link = oc
                utils.log(f"Using Auto-Detected Grounded Root (Deep Search): {self.base_link.name}")
                return
        
        if self.root.isGrounded:
            self.base_link = self.root
            utils.log(f"Using Auto-Detected Grounded Root (Root Component): {self.base_link.name}")
            return
        
        def find_grounded_in_children(occ):
            if occ.isGrounded:
                return occ
            for child in occ.childOccurrences:
                result = find_grounded_in_children(child)
                if result:
                    return result
            return None
        
        for occ in self.root.occurrences:
            result = find_grounded_in_children(occ)
            if result:
                self.base_link = result
                utils.log(f"Using Auto-Detected Grounded Root (Recursive): {self.base_link.name}")
                return
        
        for oc in self.root.allOccurrences:
            if oc.isVisible:
                self.base_link = oc
                utils.log(f"WARNING: No grounded component found. Using first visible: {self.base_link.name}")
                return
        
        utils.fatal("Failed to find any suitable root link. Please select one manually from the dropdown menu.")

    def get_name(self, oc: adsk.fusion.Occurrence) -> str:
        if oc.entityToken in self.links_by_token:
            return self.links_by_token[oc.entityToken]
        name = utils.rename_if_duplicate(self.name_map.get(oc.name, oc.name), self.links_by_name)
        formatted_name = utils.format_name(name)
        self.links_by_name[formatted_name] = oc
        self.links_by_token[oc.entityToken] = formatted_name
        utils.log(f"DEBUG: link '{oc.name}' ('{oc.fullPathName}') became '{formatted_name}'")
        return formatted_name
    
    def _get_inertia(self, occ: adsk.fusion.Occurrence) -> dict:
        """Calculates mass, COM and inertia in kg*m^2 with correct transformation."""
        # We use the new function from transforms.py
        data = transforms.inertia_from_occurrence(occ, self.inertia_accuracy)
        if data['mass'] <= 0:
            data['mass'] = 0.001
        return data

    def _iterate_through_occurrences(self) -> Iterable[adsk.fusion.Occurrence]:
        for token in self.component_map.values():
            yield token.component

    def _joints(self):
        # First, fix joint values (only for healthy joints)
        for joint in self.root.allJoints:
            # Health check BEFORE doing anything
            if joint.healthState in [adsk.fusion.FeatureHealthStates.SuppressedFeatureHealthState, adsk.fusion.FeatureHealthStates.RolledBackFeatureHealthState]:
                continue
            try:
                if isinstance(joint.jointMotion, adsk.fusion.RevoluteJointMotion):
                    if joint.jointMotion.rotationValue != 0.0:
                        utils.log(f"WARNING: joint {joint.name} was not at 0, rotating it to 0")
                        joint.jointMotion.rotationValue = 0.0
                elif isinstance(joint.jointMotion, adsk.fusion.SliderJointMotion):
                    if joint.jointMotion.slideValue != 0.0:
                        utils.log(f"WARNING: joint {joint.name} was not at 0, sliding it to 0")
                        joint.jointMotion.slideValue = 0.0
            except Exception as e:
                try:
                    o1 = joint.occurrenceOne.fullPathName
                except:
                    o1 = "Unknown"
                try:
                    o2 = joint.occurrenceTwo.fullPathName
                except:
                    o2 = "Unknown"
                utils.fatal(
                    f"Fusion errored out trying to operate on `jointMotion` of joint {joint.name}"
                    f" (between {o1} and {o2}, child of {joint.parentComponent.name}) with Health State {joint.healthState}: {e}")

        # Now, process joints for URDF
        for joint in sorted(self.root.allJoints, key=lambda joint: joint.name):
            # Health check – if not Healthy, skip it
            if joint.healthState != adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState:
                utils.log(f"WARNING: Skipping joint {joint.name} (child of {joint.parentComponent.name}) because healthState is {joint.healthState}")
                continue

            orig_name = joint.name
            try:
                _ = joint.entityToken
                joint_type = Configurator.joint_types[joint.jointMotion.jointType]
                occ_one = joint.occurrenceOne
                occ_two = joint.occurrenceTwo
            except RuntimeError as e:
                utils.log(f"WARNING: Failed to process joint {joint.name} (child of {joint.parentComponent.name}): {e}, {joint.isValid=}. This is likely a Fusion bug - the joint was likely deleted, but somehow we still see it. Will ignore it.")
                continue

            if occ_one is None or occ_two is None:
                utils.log(f"WARNING: Failed to process joint {joint.name} (child of {joint.parentComponent.name}): {joint.isValid=}: occ_one is {None if occ_one is None else occ_one.name}, occ_two is {None if occ_two is None else occ_two.name}")
                continue

            name = utils.rename_if_duplicate(self.name_map.get(joint.name, joint.name), self.joints_dict)
            parent = self.get_name(occ_one)
            child = self.get_name(occ_two)

            if utils.LOG_DEBUG:
                utils.log(f"... Processing joint {orig_name}->{name} of type {joint_type}, between {occ_one.name}->{parent} and {occ_two.name}->{child}")

            utils.log(f"DEBUG: Got from Fusion: {joint_type} {name} connecting")
            utils.log(f"DEBUG: ... {parent} @ {occ_one.transform2.translation.asArray()} and")
            utils.log(f"DEBUG: ... {child} @ {occ_two.transform2.translation.asArray()}")

            if joint_type == "fixed":
                info = JointInfo(name=name, child=child, parent=parent)
            else:
                try:
                    geom_one_origin = get_origin(joint.geometryOrOriginOne)
                except RuntimeError:
                    geom_one_origin = None
                try:
                    geom_two_origin = get_origin(joint.geometryOrOriginTwo)
                except RuntimeError:
                    geom_two_origin = None

                utils.log(f"DEBUG: ... Origin 1: {utils.vector_to_str(geom_one_origin) if geom_one_origin is not None else None}")
                utils.log(f"DEBUG: ... Origin 2: {utils.vector_to_str(geom_two_origin) if geom_two_origin is not None else None}")

                if occ_one.assemblyContext != occ_two.assemblyContext:
                    utils.log(f"DEBUG: Non-fixed joint {name} crosses the assembly context boundary:"
                                f" {parent} is in {get_context_name(occ_one.assemblyContext)}"
                                f" but {child} is in {get_context_name(occ_two.assemblyContext)}")

                if geom_one_origin is None:
                    utils.fatal(f'Non-fixed joint {orig_name} does not have an origin, aborting')
                elif geom_two_origin is not None and not self.close_enough(geom_two_origin, geom_one_origin):
                    utils.log(f'WARNING: Occurrences {occ_one.name} and {occ_two.name} of non-fixed {orig_name}' +
                                       f' have origins {geom_one_origin.asArray()} and {geom_two_origin.asArray()}'
                                       f' that do not coincide.')
                        
                if isinstance(joint.jointMotion, adsk.fusion.RevoluteJointMotion):
                    joint_vector = joint.jointMotion.rotationAxisVector
                    limits = joint.jointMotion.rotationLimits
                    if limits.isMaximumValueEnabled and limits.isMinimumValueEnabled:
                        joint_limit_max = limits.maximumValue
                        joint_limit_min = limits.minimumValue
                        continuous = False
                        if abs(joint_limit_max - joint_limit_min) < 1e-6:
                            continuous = True
                    else:
                        continuous = True
                        joint_limit_max = 3.14159
                        joint_limit_min = -3.14159
                    # If it's continuous, change the type to "continuous"
                    if continuous:
                        joint_type = "continuous"
                elif isinstance(joint.jointMotion, adsk.fusion.SliderJointMotion):
                    joint_vector = joint.jointMotion.slideDirectionVector
                    limits = joint.jointMotion.slideLimits
                    if limits.isMaximumValueEnabled and limits.isMinimumValueEnabled:
                        joint_limit_max = limits.maximumValue * self.cm
                        joint_limit_min = limits.minimumValue * self.cm
                    else:
                        joint_limit_max = 1e6
                        joint_limit_min = -1e6
                    continuous = False
                else:
                    joint_vector = adsk.core.Vector3D.create()
                    joint_limit_max = 0.0
                    joint_limit_min = 0.0
                    continuous = False

                info = JointInfo(
                    name=name, child=child, parent=parent, origin=geom_one_origin, type=joint_type,
                    axis=joint_vector, upper_limit=joint_limit_max, lower_limit=joint_limit_min,
                    continuous=continuous)

            self.joints_dict[name] = info

        processed_groups = set()
        for group in sorted(self.root.allRigidGroups, key=lambda group: group.name):
            original_group_name = group.name
            try:
                if group.isSuppressed:
                    utils.log(f"WARNING: Skipping suppressed rigid group {original_group_name} (child of {group.parentComponent.name})")
                    continue
                if not group.isValid:
                    utils.log(f"WARNING: skipping invalid rigid group {original_group_name} (child of {group.parentComponent.name})")
                    continue
            except RuntimeError as e:
                utils.log(f"WARNING: skipping invalid rigid group {original_group_name}: (child of {group.parentComponent.name}) {e}")
                continue
            utils.log(f"DEBUG: Processing rigid group {original_group_name}: {[(occ.name if occ else None) for occ in group.occurrences]}")
            parent_occ: Optional[adsk.fusion.Occurrence] = None
            for occ in group.occurrences:
                if occ is None:
                    continue
                if occ.entityToken in processed_groups:
                    continue
                if parent_occ is None:
                    parent_occ = occ
                    processed_groups.add(occ.entityToken)
                    continue
                rigid_group_occ_name = utils.rename_if_duplicate(original_group_name, self.joints_dict)
                parent_occ_name = self.get_name(parent_occ)
                occ_name = self.get_name(occ)
                utils.log(
                    f"DEBUG: Got from Fusion: {rigid_group_occ_name}, connecting",
                    f"parent {parent_occ_name} @ {utils.vector_to_str(parent_occ.transform2.translation)} and"
                    f"child {occ_name} {utils.vector_to_str(occ.transform2.translation)}")
                self.joints_dict[rigid_group_occ_name] = JointInfo(name=rigid_group_occ_name, parent=parent_occ_name, child=occ_name)
                processed_groups.add(occ.entityToken)
        
        self.assembly_tokens: Set[str] = set()
        for occ in self.root.allOccurrences:
            if occ.childOccurrences.count > 0:
                self.assembly_tokens.add(occ.entityToken)

    def get_assembly_links(self, occ: adsk.fusion.Occurrence, parent_included: bool) -> List[str]:
        result: List[str] = []
        for child in occ.childOccurrences:
            child_included = child.entityToken in self.links_by_token
            if child_included:
                result.append(self.links_by_token[child.entityToken])
            child_included = parent_included or child_included
            if child.entityToken in self.assembly_tokens:
                result += self.get_assembly_links(child, child_included)
            elif not child_included:
                result.append(self.get_name(child))
        utils.log(f"DEBUG: get_assembly_links({occ.name}) = {result}")
        return result
    
    @staticmethod
    def _mk_pattern(name: str) -> Union[str, Tuple[str,str]]:
        c = name.count("*")
        if c > 1:
            utils.fatal(f"Occurrance name pattern '{name}' is invalid: only one '*' is supported")
        if c:
            pref, suff = name.split("*", 1)
            return (pref, suff)
        return name
    
    @staticmethod
    def _match(candidate: str, pattern: Union[str, Tuple[str,str]]) -> bool:
        if isinstance(pattern, str):
            return candidate == pattern
        pref, suff = pattern
        return len(candidate) >= len(pref) + len(suff) and candidate.startswith(pref) and candidate.endswith(suff)

    def _resolve_name(self, name:str) -> adsk.fusion.Occurrence:
        if "+" in name:
            name_parts = name.split("+")
            l = len(name_parts)
            patts = [self._mk_pattern(p) for p in name_parts]
            candidate: Optional[adsk.fusion.Occurrence]= None
            for occ in self._iterate_through_occurrences():
                path = occ.fullPathName.split("+")
                if len(path) < l:
                    continue
                mismatch = False
                for (cand, patt) in zip(path[-l:], patts):
                    if not self._match(cand, patt):
                        mismatch = True
                        break
                if mismatch:
                    continue
                if candidate is None:
                    candidate = occ
                else:
                    utils.fatal(f"Name/pattern '{name}' in configuration file matches at least two occurrences: '{candidate.fullPathName}' and '{occ.fullPathName}', update to be more specific")
            if not candidate:
                utils.fatal(f"Name/pattern '{name}' in configuration file does not match any occurrences")
            return candidate
        patt = self._mk_pattern(name)
        candidates = [occ for occ in self._iterate_through_occurrences() if self._match(occ.name, patt)]
        if not candidates:
            utils.fatal(f"Name/pattern '{name}' in configuration file does not match any occurrences")
        if len(candidates) > 1:
            utils.fatal(f"Name/pattern '{name}' in configuration file matches at least two occurrences: '{candidates[0].fullPathName}' and '{candidates[1].fullPathName}', update to be more specific")
        return candidates[0]

    def _links(self):
        self.merged_links_by_link: Dict[str, Tuple[str, List[str], List[adsk.fusion.Occurrence]]] = OrderedDict()
        self.merged_links_by_name: Dict[str, Tuple[str, List[str], List[adsk.fusion.Occurrence]]] = OrderedDict()

        for name, names in self.merge_links.items():
            if not names:
                utils.fatal(f"Invalid MergeLinks YAML config setting: merged link '{name}' is empty, which is not allowed")
            link_names = []
            for n in names:
                occ = self._resolve_name(n)
                if occ.entityToken in self.links_by_token or occ.entityToken in self.assembly_tokens:
                    link_names.append(self.get_name(occ))
                if occ.entityToken in self.assembly_tokens:
                    try:
                        link_names += self.get_assembly_links(occ, occ.entityToken in self.links_by_token)
                    except ValueError as e:
                        utils.fatal(f"Invalid MergeLinks YAML config setting: assembly '{n}' for merged link '{name}' could not be processed: {e.args[0]}")
            if name in self.links_by_name and name not in names:
                utils.fatal(f"Invalid MergeLinks YAML config setting: merged '{name}' clashes with existing Fusion link '{self.links_by_name[name].fullPathName}'; add the latter to NameMap in YAML to avoid the name clash")
            link_names = list(OrderedDict.fromkeys(link_names))
            val = name, link_names, [self.links_by_name[n] for n in link_names]
            utils.log(f"Merged link {name} <- occurrences {link_names}")
            self.merged_links_by_name[name] = val
            for link_name in link_names:
                if link_name in self.merged_links_by_link:
                    utils.fatal(f"Invalid MergeLinks YAML config setting: {link_name} is included in two merged links: '{name}' and '{self.merged_links_by_link[link_name][0]}'")
                self.merged_links_by_link[link_name] = val

        body_names: Dict[str, Tuple[()]] = OrderedDict()
        
        renames = set(self.name_map)

        for oc in self._iterate_through_occurrences():
            renames.difference_update([oc.name])
            occ_name, _, occs = self._get_merge(oc)
            if occ_name in self.body_dict:
                continue

            oc_name = utils.format_name(occ_name)
            self.body_dict[oc_name] = []
            bodies = set()

            for sub_oc in occs:                
                sub_oc_name = utils.format_name(self.get_name(sub_oc))
                if sub_oc_name != oc_name:
                    sub_oc_name = f"{oc_name}__{sub_oc_name}"
                for body in self.body_mapper[sub_oc.entityToken]:
                    if body.isVisible and body.entityToken not in bodies:
                        body_name = f"{sub_oc_name}__{utils.format_name(body.name)}"
                        unique_bodyname = utils.rename_if_duplicate(body_name, body_names)
                        body_names[unique_bodyname] = ()
                        self.body_dict[oc_name].append((body, unique_bodyname))
                        bodies.add(body.entityToken)
        
        if renames:
            ValueError("Invalid NameMap YAML config setting: some of the links are not in Fusion: '" + "', '".join(renames) + "'")

    def __add_link(self, name: str, occs: List[adsk.fusion.Occurrence]):
        urdf_origin = self.link_origins[name]
        inv = urdf_origin.copy()
        assert inv.invert()

        mass = 0.0
        visible = False
        total_com = np.zeros(3)
        body_data = []
        for occ in occs:
            inertia_data = self._get_inertia(occ)
            m = inertia_data['mass']
            if m <= 0:
                continue
            com_i = np.array(inertia_data['center_of_mass'])
            I_i = inertia_data['inertia_matrix']
            total_com += com_i * m
            mass += m
            visible = visible or occ.isVisible
            body_data.append((m, com_i, I_i))
        
        if mass <= 0:
            mass = 0.001
            total_com = np.zeros(3)
            inertia_matrix = np.eye(3) * 1e-6
        else:
            total_com /= mass
            inertia_matrix = np.zeros((3,3))
            for m, com_i, I_i in body_data:
                d = com_i - total_com
                d2 = np.dot(d, d)
                I_steiner = m * (np.eye(3) * d2 - np.outer(d, d))
                inertia_matrix += I_i + I_steiner

        for i in range(3):
            if inertia_matrix[i,i] < 1e-8:
                inertia_matrix[i,i] = 1e-8

        inertia_list = [
            float(inertia_matrix[0,0]), float(inertia_matrix[1,1]), float(inertia_matrix[2,2]),
            float(inertia_matrix[0,1]), float(inertia_matrix[0,2]), float(inertia_matrix[1,2])
        ]
        
        com_local = total_com - np.array(urdf_origin.translation.asArray())
        rot = np.array([[urdf_origin.getCell(i,j) for j in range(3)] for i in range(3)])
        inertia_local = rot.T @ inertia_matrix @ rot
        
        for i in range(3):
            if inertia_local[i,i] < 1e-8:
                inertia_local[i,i] = 1e-8
        
        inertia_list_local = [
            float(inertia_local[0,0]), float(inertia_local[1,1]), float(inertia_local[2,2]),
            float(inertia_local[0,1]), float(inertia_local[0,2]), float(inertia_local[1,2])
        ]
        
        com_m = com_local.tolist()
        
        formatted_name = utils.format_name(name)
        
        if formatted_name not in self.body_dict:
            self.body_dict[formatted_name] = []
            
        self.bodies_collected.update(body.entityToken for body, _ in self.body_dict[formatted_name])

        self.links[formatted_name] = parts.Link(name = formatted_name,
                        xyz = (u * self.cm for u in inv.translation.asArray()),
                        rpy = transforms.so3_to_euler(inv),
                        center_of_mass = com_m,
                        sub_folder = self.mesh_folder,
                        mass = mass,
                        inertia_tensor = inertia_list_local,
                        bodies = [body_name for _, body_name in self.body_dict[formatted_name]],
                        sub_mesh = self.sub_mesh,
                        material_dict = self.material_dict,
                        visible = visible)

    def __get_material(self, appearance: Optional[adsk.core.Appearance]) -> str:
        if appearance is not None:
            for prop in appearance.appearanceProperties:
                if type(prop) == adsk.core.ColorProperty:
                    prop_name = appearance.name
                    color_name = utils.convert_german(prop_name)
                    color_name = utils.format_name(color_name)
                    self.color_dict[color_name] = f"{prop.value.red/255} {prop.value.green/255} {prop.value.blue/255} {prop.value.opacity/255}"
                    return color_name
        return "silver_default"

    def _materials(self) -> None:
        self.color_dict['silver_default'] = "0.700 0.700 0.700 1.000"

        if self.sub_mesh:
            for occ_name, bodies in self.body_dict.items():
                if len(bodies) > 1:
                    for body, body_name in bodies:
                        self.material_dict[body_name] = self.__get_material(body.appearance)
                else:
                    appearance = self.__get_material(bodies[0][0].appearance) if bodies else 'silver_default'
                    self.material_dict[utils.format_name(occ_name)] = appearance 
        else:
            for occ in self.links_by_name.values():
                occ_name, _, occs = self._get_merge(occ)
                occ = occs[0]
                appearance = None
                if occ.appearance:
                    appearance = occ.appearance
                elif occ.bRepBodies:
                    for body in occ.bRepBodies:
                        if body.appearance:
                            appearance = body.appearance
                            break
                elif occ.component.material:
                    appearance = occ.component.material.appearance
                self.material_dict[utils.format_name(occ_name)] = self.__get_material(appearance)

    def _get_merge(self, occ: adsk.fusion.Occurrence) -> Tuple[str, List[str], List[adsk.fusion.Occurrence]]:
        name = self.get_name(occ)
        if name in self.merged_links_by_link:
            return self.merged_links_by_link[name]
        return name, [name], [occ]

    def _build(self) -> None:
        self.link_origins: Dict[str, adsk.core.Matrix3D] = {}

        occurrences: Dict[str, List[str]] = OrderedDict()
        for joint_name, joint_info in self.joints_dict.items():
            occurrences.setdefault(joint_info.parent, [])
            occurrences.setdefault(joint_info.child, [])
            occurrences[joint_info.parent].append(joint_name)
            occurrences[joint_info.child].append(joint_name)
        for link_name, joints in occurrences.items():
            utils.log(f"DEBUG: {link_name} touches joints {joints}")
        assert self.base_link is not None
        self.base_link_name, base_link_names, base_link_occs = self._get_merge(self.base_link)
        grounded_occ = set(base_link_names)
        
        formatted_base_name = utils.format_name(self.base_link_name)
        
        for name in [formatted_base_name] + [utils.format_name(n) for n in base_link_names]:
            self.link_origins[name] = base_link_occs[0].transform2
        
        self.__add_link(formatted_base_name, base_link_occs)
        boundary = grounded_occ.copy()
        fixed_links: Dict[Tuple[str,str], str] = {}
        while boundary:
            new_boundary : Set[str] = set()
            for occ_name in boundary:
                for joint_name in occurrences.get(occ_name, ()):
                    joint = self.joints_dict[joint_name]
                    if joint.parent == occ_name:
                        child_name = joint.child
                        if child_name in grounded_occ:
                            continue
                        flip_axis = True
                    else:
                        assert joint.child == occ_name
                        if joint.parent in grounded_occ:
                            continue
                        child_name = joint.parent
                        flip_axis = False

                    parent_name, _, _ = self._get_merge(self.links_by_name[occ_name])
                    child_name, child_link_names, child_link_occs = self._get_merge(self.links_by_name[child_name])

                    child_origin = child_link_occs[0].transform2
                    parent_origin = self.link_origins[utils.format_name(parent_name)]

                    if utils.LOG_DEBUG and self.close_enough(parent_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]) and not self.close_enough(child_origin.getAsCoordinateSystem()[1:], adsk.core.Matrix3D.create().getAsCoordinateSystem()[1:]):
                        utils.log(f"***** !!!!! rotating off the global frame's orientation")
                        utils.log(f"      Child axis: {[v.asArray() for v in child_origin.getAsCoordinateSystem()[1:]]}")

                    t = parent_origin.copy()
                    assert t.invert()

                    axis = joint.axis
                    
                    if joint.type == "fixed":
                        fixed_links[(child_name, parent_name)] = joint.name
                        fixed_links[(parent_name, child_name)] = joint.name
                    else:
                        utils.log(f"DEBUG: for non-fixed joint {joint.name}, updating child origin from {utils.ct_to_str(child_origin)} to {joint.origin.asArray()}")
                        child_origin = child_origin.copy()
                        child_origin.translation = joint.origin
                        tt = child_origin.copy()
                        tt.translation = adsk.core.Vector3D.create()
                        assert tt.invert()
                        axis = axis.copy()
                        assert axis.transformBy(tt)
                        if flip_axis:
                            assert axis.scaleBy(-1)
                        utils.log(f"DEBUG:    and using {utils.ct_to_str(tt)} and {flip_axis=} to update axis from {joint.axis.asArray()} to {axis.asArray()}")

                    formatted_child_name = utils.format_name(child_name)
                    for name in [formatted_child_name] + [utils.format_name(n) for n in child_link_names]:
                        self.link_origins[name] = child_origin

                    ct = child_origin.copy()
                    assert ct.transformBy(t)

                    xyz = [c * self.cm for c in ct.translation.asArray()]
                    rpy = transforms.so3_to_euler(ct)

                    utils.log(
                        f"DEBUG: joint {joint.name} (type {joint.type})"
                        f" from {parent_name} at {utils.vector_to_str(parent_origin.translation)}"
                        f" to {child_name} at {utils.vector_to_str(child_origin.translation)}"
                        f" -> xyz={utils.vector_to_str(xyz,5)} rpy={utils.rpy_to_str(rpy)}")

                    # For continuous joints, set very large limits (will be ignored by parts.Joint)
                    if joint.type == "continuous":
                        upper = 1e6
                        lower = -1e6
                    elif joint.continuous:
                        upper = 1e6
                        lower = -1e6
                    else:
                        upper = joint.upper_limit
                        lower = joint.lower_limit

                    self.joints[joint.name] = parts.Joint(name=joint.name , joint_type=joint.type, 
                                    xyz=xyz, rpy=rpy, axis=axis.asArray(), 
                                    parent=parent_name, child=child_name, 
                                    upper_limit=upper, lower_limit=lower)
                    
                    self.__add_link(formatted_child_name, child_link_occs)
                    new_boundary.update(child_link_names)
                    grounded_occ.update(child_link_names)

            boundary = new_boundary
        
        disconnected_external = []
        for name in self.extra_links:
            if name in self.merge_links:
                name2, _, occs = self.merged_links_by_name[name]
                if name2 == self.base_link_name:
                    utils.fatal(f"Link '{name2}' is the root link, but declared as an extra (that is, not a part of the main URDF)")
                for oc in occs:
                    self.link_origins[utils.format_name(self.get_name(oc))] = occs[0].transform2
            else:
                if name == self.base_link_name:
                    utils.fatal(f"Link '{name}' is the root link, but declared as an extra (that is, not a part of the main URDF)")
                elif name not in self.links_by_name:
                    utils.fatal(f"Link '{name}' from the 'Extras:' section of the configuration file is not known")
                occs = [self.links_by_name[name]]
            formatted_name = utils.format_name(name)
            self.link_origins[formatted_name] = occs[0].transform2
            self.__add_link(formatted_name, occs)
            disconnected_external.append(formatted_name)
            

        joint_children: Dict[str, List[parts.Joint]] = OrderedDict()
        for joint in self.joints.values():
            joint_children.setdefault(joint.parent, [])
            joint_children[joint.parent].append(joint)
        tree_str = []
        def get_tree(level: int, link_name: str, exclude: Set[str]):
            extra = f" {self.merge_links[link_name]}" if link_name in self.merge_links else ""
            tree_str.append("   "*level + f" - Link: {link_name}{extra}")
            for j in joint_children.get(link_name, ()):
                if j.child not in exclude:
                    tree_str.append("   " * (level + 1) + f" - Joint [{j.type}]: {j.name}")
                    get_tree(level+2, j.child, exclude)
        get_tree(1, self.base_link_name, set(disconnected_external))
        if disconnected_external:
            tree_str.append("     - \"Extras\" links:")
            for extra in disconnected_external:
                get_tree(2, extra, set(self.link_origins).difference(disconnected_external))

        not_in_joints = set()
        unreachable = set()
        for occ in self._iterate_through_occurrences():
            if occ.isVisible and self.body_dict.get(self.name) is not None:
                if occ.fullPathName not in self.links_by_token:
                    not_in_joints.add(occ.fullPathName)
                elif self.links_by_token[occ.fullPathName] not in grounded_occ:
                    unreachable.add(occ.fullPathName)
        for occ in self.root.allOccurrences:
            if any (b.isVisible and not b.entityToken in self.bodies_collected for b in occ.bRepBodies):
                unreachable.add(occ.fullPathName)
        if not_in_joints or unreachable:
            error = "FATAL ERROR: Not all occurrences were included in the export:"
            if not_in_joints:
                error += "Not a part of any joint or rigid group: " + ", ".join(not_in_joints) + "."
            if unreachable:
                error += "Unreacheable from the grounded occurrence via joints+links: " + ", ".join(unreachable) + "."
            utils.log(error)
        missing_joints = set(self.joints_dict).difference(self.joints)
        for joint_name in missing_joints.copy():
            joint = self.joints_dict[joint_name]
            if joint.type == "fixed":
                parent_name, _, _ = self._get_merge(self.links_by_name[joint.parent])
                child_name, _, _ = self._get_merge(self.links_by_name[joint.child])
                if parent_name == child_name:
                    utils.log(f"DEBUG: Skipped Fixed Joint '{joint_name}' that is internal for merged link {self.merged_links_by_link[joint.parent][0]}")
                    missing_joints.remove(joint_name)
                elif (parent_name, child_name) in fixed_links:
                    utils.log(f"DEBUG: Skipped Fixed Joint '{joint_name}' that is duplicative of `{fixed_links[(parent_name, child_name)]}")
                    missing_joints.remove(joint_name)
        if missing_joints:
            utils.log("\n\t".join(["FATAL ERROR: Lost joints: "] + [f"{self.joints_dict[joint].name} of type {self.joints_dict[joint].type} between {self.joints_dict[joint].parent} and {self.joints_dict[joint].child}" for joint in missing_joints]))
        extra_joints = set(self.joints).difference(self.joints_dict)
        if extra_joints:
            utils.log("FATAL ERROR: Extra joints: '" + "', '".join(sorted(extra_joints)) + "'")
        if not_in_joints or unreachable or missing_joints or extra_joints:
            utils.log("Reachable from the root:")
            utils.log("\n".join(tree_str))
            utils.fatal("Fusion structure is broken or misunderstoon by the exporter, giving up! See the full output in Text Commands console for more information.")
        self.tree_str = tree_str

        for link, locations in self.locations.items():
            formatted_link = utils.format_name(link)
            if formatted_link not in self.link_origins:
                utils.fatal(f"Link {link} specified in the config file 'Locations:' section does not exist. Make sure to use the URDF name (e.g. as set via MergeLink) rather than the Fusion one")
            origin = self.link_origins[formatted_link]
            t = origin.copy()
            assert t.invert()
            self.locs[formatted_link] = []
            for loc_name, loc_occurrence in locations.items():
                if loc_occurrence in self.merge_links:
                    ct = self.link_origins[utils.format_name(loc_occurrence)].copy()
                else:
                    ct = self._resolve_name(loc_occurrence).transform2.copy()
                assert ct.transformBy(t)
                self.locs[formatted_link].append(parts.Location(loc_name, [c * self.cm for c in ct.translation.asArray()], rpy = transforms.so3_to_euler(ct)))