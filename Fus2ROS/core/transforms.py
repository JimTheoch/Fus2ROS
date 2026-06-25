# -*- coding: utf-8 -*
from typing import Tuple, List, Optional
import adsk.core
import adsk.fusion
import numpy as np
from scipy.spatial.transform import Rotation
from . import utils

def so3_to_euler(mat: adsk.core.Matrix3D) -> Tuple[float, float, float]:
    """Converts an SO3 rotation matrix to Euler angles (Roll, Pitch, Yaw)"""
    so3 = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            so3[i,j] = mat.getCell(i,j)
    r = Rotation.from_matrix(so3)
    yaw, pitch, roll = r.as_euler("ZYX", degrees=False)
    return (float(roll), float(pitch), float(yaw))

def transform_inertia_to_new_frame(
    inertia_world: List[float],
    mass: float,
    com_world: List[float],
    rot_matrix: np.ndarray
) -> List[float]:
    I = np.array([
        [inertia_world[0], inertia_world[3], inertia_world[4]],
        [inertia_world[3], inertia_world[1], inertia_world[5]],
        [inertia_world[4], inertia_world[5], inertia_world[2]]
    ])
    x, y, z = com_world
    d2 = x*x + y*y + z*z
    I_steiner = mass * np.array([
        [d2 - x*x, -x*y, -x*z],
        [-x*y, d2 - y*y, -y*z],
        [-x*z, -y*z, d2 - z*z]
    ])
    I_com = I - I_steiner
    I_new = rot_matrix @ I_com @ rot_matrix.T
    min_val = 1e-8
    for i in range(3):
        if I_new[i,i] < min_val:
            I_new[i,i] = min_val
    return [
        float(I_new[0,0]), float(I_new[1,1]), float(I_new[2,2]),
        float(I_new[0,1]), float(I_new[0,2]), float(I_new[1,2])
    ]

def inertia_from_occurrence(
    occ: adsk.fusion.Occurrence,
    accuracy: adsk.fusion.CalculationAccuracy
) -> dict:
    """
    Extracts mass, COM and the FULL inertia tensor (with off-diagonal components)
    in kg*m^2 from an Occurrence.
    Priority: getXYZMomentsOfInertia() -> getPrincipalMomentsOfInertia() -> fallback.
    """
    props = occ.getPhysicalProperties(accuracy)
    mass = props.mass
    if mass <= 0:
        mass = 0.001

    com = props.centerOfMass
    com_m = [com.x * 0.01, com.y * 0.01, com.z * 0.01]  # cm → m

    I_com = None

    # 1. FIRST ATTEMPT: getXYZMomentsOfInertia (full tensor)
    try:
        (retVal, xx, yy, zz, xy, yz, xz) = props.getXYZMomentsOfInertia()
        I_world = np.array([
            [xx * 1e-4, xy * 1e-4, xz * 1e-4],
            [xy * 1e-4, yy * 1e-4, yz * 1e-4],
            [xz * 1e-4, yz * 1e-4, zz * 1e-4]
        ])
        # Shift to center of mass (Steiner)
        d = np.array([-c for c in com_m])  # origin - COM
        d2 = np.dot(d, d)
        I_steiner = mass * (np.eye(3) * d2 - np.outer(d, d))
        I_com = I_world - I_steiner
        utils.log(f"DEBUG: getXYZMomentsOfInertia succeeded for {occ.name}")
    except Exception as e:
        utils.log(f"WARNING: getXYZMomentsOfInertia failed for {occ.name}: {e}")

    # 2. SECOND ATTEMPT: getPrincipalMomentsOfInertia (diagonal only)
    if I_com is None:
        try:
            (retVal, ixx, iyy, izz) = props.getPrincipalMomentsOfInertia()
            I_com = np.diag([ixx * 1e-4, iyy * 1e-4, izz * 1e-4])
            utils.log(f"DEBUG: getPrincipalMomentsOfInertia succeeded for {occ.name}")
        except Exception as e:
            utils.log(f"WARNING: getPrincipalMomentsOfInertia failed for {occ.name}: {e}")

    # 3. FINAL FALLBACK
    if I_com is None:
        utils.log(f"WARNING: Using fallback inertia (1e-6) for {occ.name}")
        I_com = np.eye(3) * 1e-6

    # Ensure positive diagonals
    for i in range(3):
        if I_com[i,i] < 1e-8:
            I_com[i,i] = 1e-8

    inert_m = [
        float(I_com[0,0]), float(I_com[1,1]), float(I_com[2,2]),
        float(I_com[0,1]), float(I_com[0,2]), float(I_com[1,2])
    ]

    return {
        'mass': mass,
        'center_of_mass': com_m,
        'inertia': inert_m,
        'inertia_matrix': I_com
    }