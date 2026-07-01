# Fus2ROS – Fusion 360 to ROS/URDF Exporter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.7+](https://img.shields.io/badge/python-3.7+-blue.svg)](https://www.python.org/downloads/)
https://img.shields.io/badge/License-MIT-yellow.svg
https://img.shields.io/badge/python-3.7+-blue.svg
https://img.shields.io/badge/ROS_1-Kinetic%252B-brightgreen
https://img.shields.io/badge/ROS_2-Foxy%252B-brightgreen


**Fus2ROS** is a powerful Fusion 360 add‑in that exports your CAD assemblies to URDF/Xacro files, ready for ROS, Gazebo, MoveIt, and pyBullet.
It automatically extracts joints, links, inertia, meshes, and material properties – and now generates complete ROS 1/2 packages with launch files, configuration, and all dependencies.

> **🚀 New:** The exporter now creates fully customised ROS 2 packages (rviz, Gazebo, MoveIt) with your robot’s name, replacing %ROBOT_NAME% in all templates.
> **⚠️ Note:** This add-in is under active development. Some features (particularly deep nested components) are still experimental.

---

## ✨ Current Features

- **Export URDF or Xacro** – choose plain URDF or ROS2-style Xacro
- **Automatic joint detection** – revolute, prismatic, fixed, continuous
- **Duplicate instance detection** – identifies and warns about duplicate components
- **Material and color export** – preserves appearances (embedded in URDF or external materials file)
- **Custom root link selection** – choose any component as the base of your robot
- **ROS2 package generation** – ready-to-use packages for `rviz2`, `Gazebo`, and `MoveIt`
- **pyBullet support** – generates a `hello_bullet.py` script for testing
- **Inertia and mass extraction** – with configurable precision (Low/Medium/Max)
- **STL mesh export** – with sub-mesh support for individual bodies

---

## 🧪 Experimental / Work in Progress

| Feature | Status | Notes |
|---------|--------|-------|
| Nested components (1 level) | ✅ Supported | Single-level assemblies work correctly |
| Deep nested components (2+ levels) | 🚧 In Progress | Complex hierarchies may have issues with transforms |
| Cylindrical joints | ⚠️ Unsupported | Will be exported as "Cylindrical_unsupported" |
| Ball joints | ⚠️ Unsupported | Will be exported as "Ball_unsupported" |
| Pin-slot joints | ⚠️ Unsupported | Will be exported as "PinSlot_unsupported" |

## 📦 Requirements

- **Fusion 360** (Personal or Commercial)
- **Python 3.7+** (bundled with Fusion 360)
- **Required Python packages:**
  - `scipy`
  - `pyyaml`

Install them with:

pip install scipy pyyaml

---

## 🚀 Installation
Clone or download this repository.

Copy the folder to your Fusion 360 Add-ins directory:

Windows: %APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\

macOS: ~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/

Start Fusion 360, go to Add-Ins → Add-Ins Manager → find Fus2ROS → check Run on Startup.

The exporter will appear under the Add-Ins tab.

---


## 🧠 How to Use
Open your robot assembly in Fusion 360.

Go to Add-Ins → Fus2ROS.

Select your root link (the base of your robot).

Configure the export settings:

Save Directory – where to save the package

Robot Name – name of your robot

Export Format – URDF (plain) or Xacro (ROS2-style)

Target Platform – None, pyBullet, rviz, Gazebo, or MoveIt

Mesh Resolution – Low, Medium, Max

Inertia Precision – Low, Medium, Max

Preview the joint hierarchy before exporting.

Generate – the exporter will create a complete ROS/ROS2 package or URDF file.

---
🔧 Supported Joint Types

| Type | Status |
|------|--------|
|Rigid (fixed)	| ✅ Supported |
|Revolute or continuous	| ✅ Supported |
|Slider	prismatic	| ✅ Supported |
|Cylindrical	| unsupported	⚠️ |
|Planar	| 🚧 Limited support |
|Ball	| unsupported	⚠️ |
|Pin-slot	| unsupported	⚠️ |

---

🐛 Known Issues
Deep nested components (3+ levels) may have incorrect transform calculations

Some joints with Health State errors are automatically skipped (warning logged)

Materials with non-ASCII characters (e.g., German umlauts) are converted but may have naming issues

---

🤝 Contributing
Contributions, bug reports, and feature requests are welcome! Feel free to open an issue or a pull request.

---

📄 License
This project is licensed under the MIT License – see the LICENSE file for details.

---


🌟 Acknowledgements
Fus2ROS is built upon the work of:

cadop - fusion360descriptor [Main inspiration]

syuntoku14 – original Fusion2URDF

yanshil – Fusion2PyBullet

cadop – refinements

apric0ts – improvements
