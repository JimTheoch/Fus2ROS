# Fus2ROS – Fusion 360 to ROS/URDF Exporter

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Fus2ROS** is a Fusion 360 add-in that exports your CAD models to **URDF/Xacro** files for use in **ROS, Gazebo, MoveIt, and pyBullet**. It automatically extracts joints, links, inertia, meshes, and material properties from your Fusion 360 assembly.

---

## ✨ Features

- **Export URDF or Xacro** with a single click
- **Automatic joint detection** – revolute, prismatic, fixed, continuous
- **Deep nested component support** – handles complex assemblies with multiple levels
- **Duplicate instance detection** – prevents invalid URDF structures
- **Material and color export** – preserves appearances
- **Custom root link selection** – choose any component as the base of your robot
- **ROS2 package generation** – ready-to-use packages for `rviz2`, `Gazebo`, and `MoveIt`
- **pyBullet support** – generates a `hello_bullet.py` script for testing

---

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
Fusion Joint Type	URDF Type
Rigid (fixed)	
Revolute (partly or continuous)
Slider/prismatic
(Warning) Cylindrical -> unsupported
Planar	(limited)

---

🤝 Contributing
Contributions, bug reports, and feature requests are welcome! Feel free to open an issue or a pull request.

---

📄 License
This project is licensed under the MIT License – see the LICENSE file for details.

---


🌟 Acknowledgements
Fus2ROS is built upon the work of:

syuntoku14 – original Fusion2URDF

yanshil – Fusion2PyBullet

cadop – refinements

apric0ts – improvements
