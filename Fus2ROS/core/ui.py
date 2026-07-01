''' module: user interface '''

from typing import Optional
import adsk.core, adsk.fusion, traceback

# Required libraries – if missing, give a clear message
try:
    from yaml import SafeLoader
    from scipy.spatial.transform import Rotation
except ImportError as e:
    raise ImportError(
        "The 'pyyaml' and 'scipy' libraries are required for the exporter to function.\n"
        "Please install them using:\n\n"
        "  pip install pyyaml scipy\n\n"
        "and restart Fusion 360."
    ) from e

from . import utils
from . import manager


def save_dir_dialog(ui: adsk.core.UserInterface) -> Optional[str]:
    """Display the dialog to pick the save directory"""
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'URDF Save Folder Dialog'
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return None


def get_duplicate_parents(root_component):
    """
    Find parent components that contain duplicate instances (based on component.entityToken)
    """
    duplicate_parents = set()
    
    try:
        if root_component is not None:
            duplicates = get_duplicate_instances_with_paths(root_component)
            
            for dup in duplicates:
                for inst in dup['instances']:
                    path_parts = inst['path'].split(" / ")
                    if len(path_parts) >= 2:
                        parent = " / ".join(path_parts[:-1])
                        duplicate_parents.add(parent)
    except Exception as e:
        utils.log(f"ERROR in get_duplicate_parents: {e}")
    
    return duplicate_parents


def get_duplicate_instances_with_paths(root_component):
    """
    Find all duplicate instances with their full paths,
    using component.entityToken for uniqueness (all instances of same component).
    Returns a list of duplicates, where each duplicate has:
        - 'token': component.entityToken (common for all instances)
        - 'name': name of the component
        - 'instances': list of dicts with 'occ', 'path', 'parent'
    """
    instances = {}  # key = component.entityToken, value = {name, path, occ}
    duplicates = []
    
    try:
        if root_component is not None:
            def get_full_path(occ):
                """Return the full hierarchical path of an occurrence"""
                path_parts = []
                current = occ
                while current is not None:
                    path_parts.insert(0, current.name)
                    current = current.assemblyContext
                return " / ".join(path_parts)
            
            def collect_occurrences(occ, parent_path=""):
                if occ.isVisible:
                    comp_token = occ.component.entityToken  # key for uniqueness
                    name = occ.name
                    full_path = get_full_path(occ)
                    # Store the first instance for each component token
                    if comp_token not in instances:
                        instances[comp_token] = {
                            'name': name,
                            'path': full_path,
                            'occ': occ,
                            'parent': occ.assemblyContext.name if occ.assemblyContext else "Root"
                        }
                    else:
                        # Duplicate found (same component)
                        dup_found = None
                        for d in duplicates:
                            if d['token'] == comp_token:
                                dup_found = d
                                break
                        if dup_found is None:
                            dup_found = {
                                'token': comp_token,
                                'name': name,
                                'instances': []
                            }
                            duplicates.append(dup_found)
                        # Add the current instance
                        dup_found['instances'].append({
                            'occ': occ,
                            'path': full_path,
                            'parent': occ.assemblyContext.name if occ.assemblyContext else "Root"
                        })
                # Continue collecting children
                for child in occ.childOccurrences:
                    collect_occurrences(child)
            
            for occ in root_component.occurrences:
                collect_occurrences(occ)
            
            # For each duplicate, prepend the first instance to the list
            for dup in duplicates:
                token = dup['token']
                first_inst = instances[token]
                dup['instances'].insert(0, {
                    'occ': first_inst['occ'],
                    'path': first_inst['path'],
                    'parent': first_inst['parent']
                })
                
    except Exception as e:
        utils.log(f"ERROR in get_duplicate_instances_with_paths: {e}")
    
    return duplicates


def get_components_to_hide(root_component):
    """
    Find all components that should be hidden from the dropdown:
    - All occurrences that are duplicates (i.e., component appears more than once)
    - Also hide all children (nested components) of duplicate occurrences
    """
    hidden_tokens = set()
    
    try:
        if root_component is not None:
            duplicates = get_duplicate_instances_with_paths(root_component)
            
            # Collect all duplicate occurrence tokens (including all instances)
            duplicate_occ_tokens = set()
            for dup in duplicates:
                for inst in dup['instances']:
                    duplicate_occ_tokens.add(inst['occ'].entityToken)
            
            # Function to collect all descendants of an occurrence
            def collect_descendants(occ, hidden_set):
                # Add this occurrence's entityToken
                hidden_set.add(occ.entityToken)
                # Recurse into children
                for child in occ.childOccurrences:
                    collect_descendants(child, hidden_set)
            
            # For each duplicate occurrence, hide it and all its descendants
            for token in duplicate_occ_tokens:
                # Find the occurrence object from the first duplicate
                # (we can use the first duplicate's first instance as reference)
                # Actually we need to find the occurrence from the token.
                # We'll traverse the tree to find it.
                pass  # We'll do this in the main traversal below
            
            # Instead, during traversal we'll hide duplicates and their children
            # We'll build a set of all tokens to hide by traversing the tree
            def mark_hidden(occ, duplicate_tokens):
                # If this occurrence is a duplicate, hide it and all descendants
                if occ.entityToken in duplicate_tokens:
                    # Add this occurrence and all descendants
                    def add_all_descendants(o):
                        hidden_tokens.add(o.entityToken)
                        for child in o.childOccurrences:
                            add_all_descendants(child)
                    add_all_descendants(occ)
                    return
                # Otherwise, check children
                for child in occ.childOccurrences:
                    mark_hidden(child, duplicate_tokens)
            
            # Start marking from root occurrences
            for occ in root_component.occurrences:
                mark_hidden(occ, duplicate_occ_tokens)
                
    except Exception as e:
        utils.log(f"ERROR in get_components_to_hide: {e}")
    
    return hidden_tokens


def get_all_components_with_hierarchy(root_component):
    """
    Return all components with their hierarchical structure,
    BUT only those that should NOT be hidden (i.e., not duplicate instances and not descendants of duplicates)
    """
    components = []
    seen = set()
    
    try:
        if root_component is not None:
            hidden_tokens = get_components_to_hide(root_component)
            
            def get_indent_level(occ, level=0):
                if occ.assemblyContext is None:
                    return level
                return get_indent_level(occ.assemblyContext, level + 1)
            
            def get_full_path(occ):
                path_parts = []
                current = occ
                while current is not None:
                    path_parts.insert(0, current.name)
                    current = current.assemblyContext
                return " / ".join(path_parts)
            
            def collect_occurrences(occ, prefix=""):
                if occ.isVisible:
                    # Check if this occurrence or any of its ancestors is hidden
                    # We use the hidden_tokens set that includes duplicates and their descendants
                    if occ.entityToken in hidden_tokens:
                        # Skip this occurrence and do NOT traverse its children
                        # (since they are already marked hidden or will be skipped)
                        return
                    
                    name = occ.name
                    indent = "  " * get_indent_level(occ)
                    if occ.assemblyContext:
                        display_name = f"{indent}└─ {name}"
                    else:
                        display_name = f"{indent}📁 {name}"
                    
                    if display_name not in seen:
                        components.append({
                            'display': display_name,
                            'name': name,
                            'occ': occ,
                            'level': get_indent_level(occ),
                            'path': get_full_path(occ),
                            'is_duplicate': False
                        })
                        seen.add(display_name)
                
                # Traverse children only if this occurrence is not hidden
                # (but we already return early if hidden)
                for child in occ.childOccurrences:
                    collect_occurrences(child)
            
            for occ in root_component.occurrences:
                collect_occurrences(occ)
                
    except Exception as e:
        utils.log(f"ERROR in get_all_components_with_hierarchy: {e}")
    
    return components


def get_duplicate_instances(root_component):
    """
    Legacy function for backward compatibility
    """
    duplicates = get_duplicate_instances_with_paths(root_component)
    result = []
    for dup in duplicates:
        result.append({
            'name': dup['name'],
            'count': len(dup['instances']),
            'instances': [d['occ'] for d in dup['instances']]
        })
    return result


def has_duplicate_instances(root_component):
    """
    Check if there are duplicate instances
    """
    duplicates = get_duplicate_instances_with_paths(root_component)
    return len(duplicates) > 0


def is_valid_root_selected(root_dropdown):
    """
    Check if a valid root component is selected (not the placeholder)
    """
    if root_dropdown is None:
        return False
    selected = root_dropdown.selectedItem
    if selected is None:
        return False
    if selected.name == "==> MUST CHOOSE ONE <==":
        return False
    return True


class MyInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self, ui: adsk.core.UserInterface):
        self.ui = ui
        super().__init__()

    def notify(self, eventArgs: adsk.core.InputChangedEventArgs) -> None:
        try:
            cmd_event = eventArgs.firingEvent
            cmd = cmd_event.sender
            assert isinstance(cmd, adsk.core.Command)
            inputs = cmd.commandInputs
            cmdInput = eventArgs.input

            directory_path = inputs.itemById('directory_path')
            robot_name = inputs.itemById('robot_name')
            save_mesh = inputs.itemById('save_mesh')
            sub_mesh = inputs.itemById('sub_mesh')
            mesh_resolution = inputs.itemById('mesh_resolution')
            inertia_precision = inputs.itemById('inertia_precision')
            target_units = inputs.itemById('target_units')
            target_platform = inputs.itemById('target_platform')
            ros_version = inputs.itemById('ros_version')  # NEW
            preview_group = inputs.itemById('preview_group')
            root_dropdown = inputs.itemById('root_component')
            selected_info = inputs.itemById('selected_info')
            preview_btn = inputs.itemById('preview')
            generate_btn = inputs.itemById('generate')

            utils.log(f"DEBUG: UI: processing command: {cmdInput.id}")

            assert isinstance(directory_path, adsk.core.TextBoxCommandInput)
            assert isinstance(robot_name, adsk.core.TextBoxCommandInput)
            assert isinstance(save_mesh, adsk.core.BoolValueCommandInput)
            assert isinstance(sub_mesh, adsk.core.BoolValueCommandInput)
            assert isinstance(mesh_resolution, adsk.core.DropDownCommandInput)
            assert isinstance(inertia_precision, adsk.core.DropDownCommandInput)
            assert isinstance(target_units, adsk.core.DropDownCommandInput)
            assert isinstance(target_platform, adsk.core.DropDownCommandInput)
            assert isinstance(ros_version, adsk.core.DropDownCommandInput)  # NEW
            assert isinstance(preview_group, adsk.core.GroupCommandInput)

            has_dups = has_duplicate_instances(manager.Manager.root)
            valid_root = is_valid_root_selected(root_dropdown)
            
            if has_dups or not valid_root:
                if preview_btn:
                    preview_btn.isEnabled = False
                if generate_btn:
                    generate_btn.isEnabled = False
            else:
                if preview_btn:
                    preview_btn.isEnabled = True
                if generate_btn:
                    generate_btn.isEnabled = True

            if cmdInput.id == 'generate':
                if has_dups:
                    self.ui.messageBox('Cannot generate while duplicate instances exist.\nPlease fix them first (Right-click -> Make Independent)')
                    return
                if not valid_root:
                    self.ui.messageBox('Please select a valid root link from the dropdown menu first.')
                    return
                    
                selected_root = root_dropdown.selectedItem.name
                if "└─" in selected_root:
                    selected_root = selected_root.split("└─")[1].strip()
                elif "📁" in selected_root:
                    selected_root = selected_root.replace("📁", "").strip()
                
                document_manager = manager.Manager(
                    directory_path.text, 
                    robot_name.text,
                    save_mesh.value, 
                    sub_mesh.value,
                    mesh_resolution.selectedItem.name, 
                    inertia_precision.selectedItem.name, 
                    target_units.selectedItem.name, 
                    target_platform.selectedItem.name,
                    ros_version.selectedItem.name,  # NEW
                    None,
                    selected_root
                )
                
                document_manager.run()

            elif cmdInput.id == 'preview':
                if has_dups:
                    self.ui.messageBox('Cannot preview while duplicate instances exist.\nPlease fix them first (Right-click -> Make Independent)')
                    return
                if not valid_root:
                    self.ui.messageBox('Please select a valid root link from the dropdown menu first.')
                    return
                    
                selected_root = root_dropdown.selectedItem.name
                if "└─" in selected_root:
                    selected_root = selected_root.split("└─")[1].strip()
                elif "📁" in selected_root:
                    selected_root = selected_root.replace("📁", "").strip()
                    
                document_manager = manager.Manager(
                    directory_path.text, 
                    robot_name.text, 
                    save_mesh.value, 
                    sub_mesh.value, 
                    mesh_resolution.selectedItem.name, 
                    inertia_precision.selectedItem.name, 
                    target_units.selectedItem.name, 
                    target_platform.selectedItem.name,
                    ros_version.selectedItem.name,  # NEW
                    None,
                    selected_root
                )
                    
                _joints = document_manager.preview()

                joints_text = inputs.itemById('jointlist')
                assert isinstance(joints_text, adsk.core.TextBoxCommandInput)

                _txt = 'joint name: parent link-> child link\n'

                for k, j in _joints.items():
                    _txt += f'{k} : {j.parent} -> {j.child}\n' 
                joints_text.text = _txt
                preview_group.isExpanded = True

            elif cmdInput.id == 'save_dir':
                config_file = save_dir_dialog(self.ui)
                if config_file is not None:
                    directory_path.text = config_file
                    directory_path.numRows = 2

            elif cmdInput.id == 'root_component':
                if root_dropdown and selected_info:
                    selected = root_dropdown.selectedItem
                    if selected and selected.name != "==> MUST CHOOSE ONE <==":
                        components = get_all_components_with_hierarchy(manager.Manager.root)
                        for comp in components:
                            if comp['display'] == selected.name:
                                selected_info.text = f"📍 Location: {comp['path']}"
                                break
                    else:
                        selected_info.text = "📍 Please select a valid component"

        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class MyDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self, ui):
        self.ui = ui
        super().__init__()
    
    def notify(self, eventArgs: adsk.core.CommandEventArgs) -> None:
        try:
            adsk.terminate()
        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class MyCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, ui: adsk.core.UserInterface, handlers):
        self.ui = ui
        self.handlers = handlers
        super().__init__()

    def notify(self, eventArgs: adsk.core.CommandCreatedEventArgs) -> None:
        try:
            cmd = eventArgs.command
            onDestroy = MyDestroyHandler(self.ui)
            cmd.destroy.add(onDestroy)
            
            onInputChanged = MyInputChangedHandler(self.ui)
            cmd.inputChanged.add(onInputChanged)
            
            self.handlers.append(onDestroy)
            self.handlers.append(onInputChanged)
            inputs = cmd.commandInputs

            assert manager.Manager.root is not None

            # Save Directory
            directory_path = inputs.addTextBoxCommandInput('directory_path', 'Save Directory', 'C:', 2, True)
            btn = inputs.addBoolValueInput('save_dir', 'Set Save Directory', False)
            btn.isFullWidth = True

            # --- DROPDOWN MENU FOR ROOT COMPONENT (HIERARCHICAL) ---
            root_group = inputs.addGroupCommandInput('root_group', 'Root Link Selection')
            root_children = root_group.children
            
            legend_text = root_children.addTextBoxCommandInput(
                'legend_text', 
                '', 
                '📁 = Top-level component  │  └─ = Nested component', 
                2, 
                True
            )
            legend_text.isFullWidth = True
            
            components_data = get_all_components_with_hierarchy(manager.Manager.root)
            
            root_dropdown = root_children.addDropDownCommandInput(
                'root_component', 
                'Select Root Link:', 
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            
            placeholder = root_dropdown.listItems.add("==> MUST CHOOSE ONE <==", True, '')
            placeholder.isSelected = True
            
            for comp in components_data:
                root_dropdown.listItems.add(comp['display'], False, '')
            
            selected_info = root_children.addTextBoxCommandInput(
                'selected_info', 
                '', 
                '📍 Please select a valid component', 
                1, 
                True
            )
            selected_info.isFullWidth = True
            
            info_text = root_children.addTextBoxCommandInput(
                'root_info', 
                '', 
                '▶▶▶ Select the root link for your URDF. This will be the base of your robot.', 
                2, 
                True
            )
            info_text.isFullWidth = True
            # -----------------------------------------

            # --- DUPLICATE INSTANCES CHECK (WITH PATHS) ---
            duplicates = get_duplicate_instances_with_paths(manager.Manager.root)
            has_dups = len(duplicates) > 0
            
            dup_group = inputs.addGroupCommandInput('duplicate_group', '⚠️ Duplicate Instances Check [ Will not be shown in Root Link Selection ]')
            dup_children = dup_group.children
            
            if duplicates:
                total_duplicates = sum(len(d['instances']) for d in duplicates)
                
                full_text = f'❌ Found {len(duplicates)} components with {total_duplicates} total instances\n\n'
                full_text += "Instances that need to be made independent:\n"
                
                for dup in duplicates:
                    full_text += f"  • {dup['name']} ({len(dup['instances'])} instances)\n"
                    for inst in dup['instances']:
                        full_text += f"      └─ {inst['path']}\n"
                
                duplicate_parents = get_duplicate_parents(manager.Manager.root)
                if duplicate_parents:
                    parent_count = len(duplicate_parents)
                    full_text += f"\n🔴 ERROR FIX: Right-click on {parent_count - 1} out of {parent_count} subassemblies and select 'Make Independent' (Keep ONE original):\n"
                    for parent in sorted(duplicate_parents):
                        dup_count = 0
                        for dup in duplicates:
                            for inst in dup['instances']:
                                if parent in inst['path']:
                                    dup_count += 1
                        full_text += f"      ▶ {parent} (contains {dup_count} duplicate components)\n"
                    full_text += "⚠️ Preview and Generate are disabled until all duplicates are fixed"
                else:
                    full_text += "\n🔴 ERROR FIX: Right-click on each instance and select 'Make Independent' (Keep ONE original)\n⚠️ Preview and Generate are disabled until all duplicates are fixed"
                
                line_count = full_text.count('\n') + 1
                text_height = max(3, min(30, line_count))
                
                dup_children.addTextBoxCommandInput(
                    'dup_full', 
                    '', 
                    full_text, 
                    text_height, 
                    True
                )
            else:
                dup_children.addTextBoxCommandInput(
                    'dup_count', 
                    '', 
                    '✅ No duplicate instances found!', 
                    1, 
                    True
                )
            # -----------------------------------------

            # Robot Name
            inputs.addTextBoxCommandInput('robot_name', 'Robot Name', manager.Manager.root.name.split()[0], 1, False)
            
            # Mesh Options
            inputs.addBoolValueInput('save_mesh', 'Save Mesh', True)
            inputs.addBoolValueInput('sub_mesh', 'Per-Body Visual Mesh', True)

            # Mesh Resolution
            di = inputs.addDropDownCommandInput('mesh_resolution', 'Mesh Resolution', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('Low', True, '')
            di.add('Medium', False, '')
            di.add('Max', False, '')

            # Inertia Precision
            di = inputs.addDropDownCommandInput('inertia_precision', 'Inertia Precision', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('Low', True, '')
            di.add('Medium', False, '')
            di.add('Max', False, '')

            # Target Units
            di = inputs.addDropDownCommandInput('target_units', 'Target Units', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('mm', False, '')
            di.add('cm', False, '')
            di.add('m', True, '')

            # ROS Version (NEW)
            di = inputs.addDropDownCommandInput('ros_version', 'ROS Version', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('ROS 2', True, '')
            di.add('ROS 1', False, '')

            # Target Platform
            di = inputs.addDropDownCommandInput('target_platform', 'Target Platform', adsk.core.DropDownStyles.TextListDropDownStyle)
            di = di.listItems
            di.add('URDF', True, '')
            di.add('Xacro', False, '')
            di.add('pyBullet', False, '')
            di.add('rviz', False, '')
            di.add('Gazebo', False, '')
            di.add('MoveIt', False, '')

            # --- PREVIEW BUTTON ---
            preview_btn = inputs.addBoolValueInput('preview', 'Preview Links', False)
            preview_btn.isFullWidth = True
            preview_btn.isEnabled = False

            # Preview Tab
            tab_input = inputs.addTabCommandInput('tab_preview', 'Preview Tabs')
            tab_input_child = tab_input.children
            preview_group = tab_input_child.addGroupCommandInput("preview_group", "Preview")
            preview_group.isExpanded = False
            textbox_group = preview_group.children

            txtbox = textbox_group.addTextBoxCommandInput('jointlist', 'Joint List', '', 8, True)
            txtbox.isFullWidth = True

            # --- GENERATE BUTTON ---
            generate_btn = inputs.addBoolValueInput('generate', 'Generate', False)
            generate_btn.isFullWidth = True
            generate_btn.isEnabled = False

            cmd.setDialogSize(500, 0)
            directory_path.numRows = 1

        except:
            if self.ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def config_settings(ui: adsk.core.UserInterface, ui_handlers) -> bool:
    try:
        commandId = 'Joint Configuration Descriptor'
        commandDescription = 'Settings to describe a URDF file'
        commandName = 'URDF Description App'

        cmdDef = ui.commandDefinitions.itemById(commandId)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(commandId, commandName, commandDescription)

        onCommandCreated = MyCreatedHandler(ui, ui_handlers)
        cmdDef.commandCreated.add(onCommandCreated)
        ui_handlers.append(onCommandCreated)

        cmdDef.execute()
        adsk.autoTerminate(False)
        return True 
    except:
        exn = traceback.format_exc()
        utils.log(f"FATAL: {exn}")
        if ui:
            ui.messageBox(f'Failed:\n{exn}')
        return False