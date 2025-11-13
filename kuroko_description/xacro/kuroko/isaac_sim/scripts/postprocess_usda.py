#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
postprocess_usda.py

Apply required post-edits to a generated USDA file (kuroko.usda), referencing
SDF for max joint velocities (rad/s -> deg/s). This script prefers the USD
Python API (pxr). If attributes don't exist in the schema, it creates custom
attributes so downstream tools (e.g., Omniverse/PhysX) can still read them.

Example:
    python3 postprocess_usda.py --usda export/kuroko.usda --sdf export/kuroko.sdf --robot /kuroko
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

# Try to use USD Python API if available
try:
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    PXRC_AVAILABLE = True
except Exception:
    PXRC_AVAILABLE = False


def parse_args():
    ap = argparse.ArgumentParser(description="Post-process a USDA file for Kuroko robot.")
    ap.add_argument("--usda", required=True, help="Path to target USDA file (ASCII).")
    ap.add_argument("--sdf", required=False, help="Path to reference SDF file (for joint velocity limits).")
    ap.add_argument("--robot", default="/kuroko", help="Prim path of the robot root (default: /kuroko).")
    return ap.parse_args()


def read_sdf_joint_vel_limits(sdf_path):
    """
    Parse SDF and return dict: { joint_name: velocity_limit_deg_per_sec }
    """
    limits = {}
    if not sdf_path:
        return limits
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        # Namespaces handling (SDF may or may not have default ns)
        # We'll ignore namespaces by stripping them.
        for j in root.iter():
            if j.tag.endswith("joint") and "name" in j.attrib:
                name = j.attrib["name"]
                vel = None
                for lim in j.iter():
                    if lim.tag.endswith("limit"):
                        for child in lim.iter():
                            if child.tag.endswith("velocity"):
                                try:
                                    vel = float(child.text.strip())
                                except Exception:
                                    pass
                if vel is not None:
                    # rad/s -> deg/s
                    limits[name] = vel * 180.0 / 3.1417
        return limits
    except Exception as e:
        print(f"[WARN] Failed to parse SDF '{sdf_path}': {e}")
        return limits


# -------- Helpers using pxr (preferred path) -------- #

def set_stage_units(stage):
    # Root Layer
    # World Axis: Z
    # Meters Per Unit: 1.0
    # Kgs Per Unit: 1.0
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    try:
        # Available in USD Physics schema
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    except Exception:
        pass


def ensure_attr(prim, name, type_name, default_value):
    """
    Create or get attribute on prim, setting default if empty.
    type_name: Sdf.ValueTypeNames token, e.g., Sdf.ValueTypeNames.Bool
    """
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    if not attr.HasAuthoredValueOpinion():
        attr.Set(default_value)
    return attr


def set_robot_body_properties(prim):
    # kuroko (robot root prim)
    # Solver Position Iteration Count: 32
    # Solver Velocity Iteration Count: 1
    # Sleep Threshold: 0.001
    # Stabilization Threshold: 0.0001
    # Self Collision Enabled: False
    ensure_attr(prim, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(prim, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)
    ensure_attr(prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)
    ensure_attr(prim, "physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float, 0.0001).Set(0.0001)
    ensure_attr(prim, "physxRigidBody:selfCollision", Sdf.ValueTypeNames.Bool, False).Set(False)


def clamp_diagonal_inertia(prim):
    """
    If physics:diagonalInertia exists and any component < 0.0001, clamp to 0.0001.
    """
    attr = prim.GetAttribute("physics:diagonalInertia")
    if not attr:
        return False
    val = attr.Get()
    if val is None:
        return False
    # val could be Gf.Vec3f/d, normalize to float tuple
    try:
        x, y, z = float(val[0]), float(val[1]), float(val[2])
    except Exception:
        return False
    changed = False
    eps = 1e-4
    nx = max(x, eps)
    ny = max(y, eps)
    nz = max(z, eps)
    if (nx, ny, nz) != (x, y, z):
        attr.Set(Gf.Vec3f(nx, ny, nz))
        changed = True
    return changed


def set_link_defaults(prim):
    # For each link:
    # Linear Damping: 0.01
    # Angular Damping: 0.01
    # Max Linear Velocity: 10.0
    # Max Angular Velocity: 1000.0
    # Sleep threshold: 0.001
    ensure_attr(prim, "physics:linearDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physics:angularDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, 10.0).Set(10.0)
    ensure_attr(prim, "physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float, 1000.0).Set(1000.0)
    ensure_attr(prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)
    clamp_diagonal_inertia(prim)


def is_joint_prim(prim):
    # USD Physics joint prims usually have types like "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", etc.
    t = prim.GetTypeName()
    if not t:
        return False
    return t.lower().startswith("physics") and t.lower().endswith("joint")


def remove_drive_components(prim):
    """
    Remove any DriveAPI instances on a joint prim, if present.
    """
    try:
        # Newer USD exposes applied API schemas list
        applied = prim.GetAppliedSchemas()
        # Instances are represented like "UsdPhysicsDriveAPI:angular" etc.
        for api in list(applied):
            if "DriveAPI" in api:
                # Extract instance name if any
                instance = None
                if ":" in api:
                    instance = api.split(":", 1)[1]
                try:
                    prim.RemoveAPI(UsdPhysics.DriveAPI, instance)
                except Exception:
                    # As a fallback, clear common drive attrs
                    for n in [
                        "drive:stiffness",
                        "drive:damping",
                        "drive:maxForce",
                        "drive:targetVelocity",
                        "drive:targetPosition",
                    ]:
                        a = prim.GetAttribute(n)
                        if a:
                            a.Clear()
    except Exception:
        # Fallback: best-effort clear common attributes
        for n in [
            "drive:stiffness",
            "drive:damping",
            "drive:maxForce",
            "drive:targetVelocity",
            "drive:targetPosition",
        ]:
            a = prim.GetAttribute(n)
            if a:
                a.Clear()


def set_exclude_from_articulation(prim, value=True):
    # Common attribute used in Omniverse for creating weld/loop joints outside articulation
    ensure_attr(prim, "physxJoint:excludeFromArticulation", Sdf.ValueTypeNames.Bool, value).Set(value)


def set_joint_drive_params(prim, max_vel_deg_per_sec):
    """
    For joints where Drive remains:
      - Max Joint Velocity: from SDF (deg/s)
      - Drive Stiffness: 100 * MaxForce
      - Drive Damping: 10 * Stiffness / MaxJointVelocity
    Looks for common attribute names; creates them if missing.
    """
    # Max Joint Velocity
    ensure_attr(prim, "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, max_vel_deg_per_sec).Set(max_vel_deg_per_sec)

    # Read or create drive:maxForce
    max_force_attr = ensure_attr(prim, "drive:maxForce", Sdf.ValueTypeNames.Float, 0.0)
    max_force = max_force_attr.Get() or 0.0
    stiffness = 100.0 * max_force
    max_v = max(1e-6, float(max_vel_deg_per_sec))
    damping = 10.0 * stiffness / max_v

    ensure_attr(prim, "drive:stiffness", Sdf.ValueTypeNames.Float, stiffness).Set(stiffness)
    ensure_attr(prim, "drive:damping", Sdf.ValueTypeNames.Float, damping).Set(damping)


def add_physics_scene(stage):
    """
    Add or update a /World/PhysicsScene prim with requested settings.
    """
    # Ensure /World xform exists
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    scene_path = Sdf.Path("/World/PhysicsScene")
    if not stage.GetPrimAtPath(scene_path):
        scene = UsdPhysics.Scene.Define(stage, scene_path)
    else:
        scene = UsdPhysics.Scene(stage.GetPrimAtPath(scene_path))

    prim = scene.GetPrim()

    # Attributes (PhysX/Omni style custom attrs where USD Physics lacks direct fields)
    ensure_attr(prim, "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, False).Set(False)
    ensure_attr(prim, "physxScene:collisionSystem", Sdf.ValueTypeNames.Token, "PCM").Set("PCM")
    ensure_attr(prim, "physxScene:solverType", Sdf.ValueTypeNames.Token, "PGS").Set("PGS")
    ensure_attr(prim, "physxScene:broadphaseType", Sdf.ValueTypeNames.Token, "GPU").Set("GPU")
    ensure_attr(prim, "physxScene:maxPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(prim, "physxScene:maxVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)
    ensure_attr(prim, "physxScene:minPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(prim, "physxScene:minVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)
    ensure_attr(prim, "physics:timeStepsPerSecond", Sdf.ValueTypeNames.Float, 200.0).Set(200.0)


def pxr_main(usda_path, sdf_path, robot_path):
    stage = Usd.Stage.Open(usda_path)
    if not stage:
        print(f"[ERROR] Failed to open USDA: {usda_path}")
        sys.exit(2)

    set_stage_units(stage)

    # Find robot root prim (e.g., /kuroko)
    robot = stage.GetPrimAtPath(robot_path)
    if not robot or not robot.IsValid():
        # Try find by name anywhere
        for p in stage.Traverse():
            if p.GetName() == robot_path.strip("/"):
                robot = p
                break

    if not robot or not robot.IsValid():
        print(f"[ERROR] Robot prim not found at '{robot_path}'")
        sys.exit(3)

    # Set robot-level body properties
    set_robot_body_properties(robot)

    # Parse SDF joint velocity limits (deg/s)
    joint_vel_deg = read_sdf_joint_vel_limits(sdf_path) if sdf_path else {}

    # Collect stats
    link_count = 0
    joint_count = 0
    removed_drive = 0
    excluded_loops = 0
    updated_drive = 0
    clamped_inertia = 0

    # Iterate children under robot
    for prim in stage.Traverse():
        # Only process within robot subtree
        if not str(prim.GetPath()).startswith(str(robot.GetPath())):
            continue

        # Links: typically Xforms or RigidBody prims with collision/mesh children
        if prim.GetTypeName() in ("Xform", "PhysicsRigidBody", "GeomSubset", "Mesh"):
            set_link_defaults(prim)
            # Count "links" loosely as Xform or PhysicsRigidBody
            if prim.GetTypeName() in ("Xform", "PhysicsRigidBody"):
                link_count += 1
                if clamp_diagonal_inertia(prim):
                    clamped_inertia += 1

        # Joints
        if is_joint_prim(prim):
            joint_count += 1
            name = prim.GetName()

            # passive / loop suffix handling
            if name.endswith("passive") or name.endswith("_passive"):
                remove_drive_components(prim)
                removed_drive += 1
            elif name.endswith("loop") or name.endswith("_loop"):
                remove_drive_components(prim)
                removed_drive += 1
                set_exclude_from_articulation(prim, True)
                excluded_loops += 1
            else:
                # Drive remains → set params from SDF velocity if available
                max_v = joint_vel_deg.get(name)
                if max_v is None:
                    # try without suffix variants
                    base = name.split(":")[-1]
                    max_v = joint_vel_deg.get(base)
                if max_v is not None:
                    set_joint_drive_params(prim, max_v)
                    updated_drive += 1

    # Add/Update PhysicsScene
    add_physics_scene(stage)

    # Save
    stage.GetRootLayer().Save()

    print("=== Post-process summary ===")
    print(f"Robot prim       : {robot.GetPath()}")
    print(f"Links processed  : {link_count} (clamped inertia on {clamped_inertia})")
    print(f"Joints processed : {joint_count}")
    print(f"Drives removed   : {removed_drive}")
    print(f"Loop joints excl.: {excluded_loops}")
    print(f"Drives updated   : {updated_drive}")
    print("Root Layer:")
    print("  Up Axis        : Z")
    print("  Meters/Unit    : 1.0")
    print("  Kilograms/Unit : 1.0 (if physics schema available)")
    print("PhysicsScene:")
    print("  GPU Dynamics   : False")
    print("  Collision Sys  : PCM")
    print("  Solver Type    : PGS")
    print("  Broadphase     : GPU")
    print("  Pos/Vel iters  : 32 / 1 (max), 32 / 1 (min)")
    print("  Steps/sec      : 200")


def main():
    args = parse_args()

    if not PXRC_AVAILABLE:
        print("[ERROR] USD Python API (pxr) is not available in this environment.")
        print("        Please run in an environment with USD installed (pxr module).")
        print("        E.g., Omniverse Kit Python or USD build from Pixar.")
        sys.exit(1)

    pxr_main(args.usda, args.sdf, args.robot)


if __name__ == "__main__":
    main()
