#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import xml.etree.ElementTree as ET

try:
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
    PXRC_AVAILABLE = True
except Exception:
    PXRC_AVAILABLE = False


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usda", required=True, help="Path to input USDA file")
    ap.add_argument("--sdf", required=False, help="Path to SDF for joint velocity limits (rad/s)")
    ap.add_argument("--robot", default="/kuroko", help="Robot root prim path (default: /kuroko)")
    return ap.parse_args()


def read_sdf_joint_vel_limits(sdf_path):
    """Return dict: joint_name -> max_velocity [deg/s] parsed from SDF (<limit><velocity> in rad/s)."""
    limits = {}
    if not sdf_path:
        return limits
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        for j in root.iter():
            if j.tag.endswith("joint") and "name" in j.attrib:
                name = j.attrib["name"]
                vel = None
                for lim in j.iter():
                    if lim.tag.endswith("limit"):
                        for child in lim.iter():
                            if child.tag.endswith("velocity") and child.text:
                                try:
                                    vel = float(child.text.strip())
                                except Exception:
                                    pass
                if vel is not None:
                    limits[name] = vel * 180.0 / 3.1417  # rad/s -> deg/s
    except Exception as e:
        print(f"[WARN] SDF parse failed: {e}")
    return limits


def ensure_attr(prim, name, type_name, default_value=None):
    """Create attribute if missing; set default if no authored opinion."""
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    if default_value is not None and not attr.HasAuthoredValueOpinion():
        attr.Set(default_value)
    return attr


def set_stage_units(stage):
    """Set upAxis=Z, metersPerUnit=1.0, kilogramsPerUnit=1.0 with fallbacks."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # metersPerUnit
    try:
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    except Exception:
        try:
            layer = stage.GetRootLayer()
            layer.SetField(Sdf.Path.emptyPath, "metersPerUnit", 1.0)
        except Exception:
            pass
    # kilogramsPerUnit
    try:
        setter = getattr(UsdPhysics, "SetStageKilogramsPerUnit", None)
        if callable(setter):
            setter(stage, 1.0)
        else:
            layer = stage.GetRootLayer()
            layer.SetField(Sdf.Path.emptyPath, "kilogramsPerUnit", 1.0)
    except Exception:
        try:
            layer = stage.GetRootLayer()
            layer.SetField(Sdf.Path.absoluteRootPath, "kilogramsPerUnit", 1.0)
        except Exception:
            print("[WARN] kilogramsPerUnit not set (unsupported build).")


def set_robot_level_attrs(robot_prim):
    """Apply articulation/rigid-body level defaults on the robot root prim."""
    # Iterations (OK in your result, keep as-is)
    ensure_attr(robot_prim, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(robot_prim, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)

    # Ensure ArticulationRootAPI is applied
    try:
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    except Exception:
        pass

    # Articulation-level attributes (fixed here)
    ensure_attr(robot_prim, "physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)
    ensure_attr(robot_prim, "physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float, 0.0001).Set(0.0001)
    ensure_attr(robot_prim, "physxArticulation:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, False).Set(False)

    # (Keep rigid-body copies for UI robustness)
    ensure_attr(robot_prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)
    ensure_attr(robot_prim, "physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float, 0.0001).Set(0.0001)


def clamp_diagonal_inertia(prim):
    """Clamp physics:diagonalInertia components to >= 1e-4 if present."""
    attr = prim.GetAttribute("physics:diagonalInertia")
    if not attr:
        return False
    val = attr.Get()
    if val is None:
        return False
    try:
        x, y, z = float(val[0]), float(val[1]), float(val[2])
    except Exception:
        return False
    changed = False
    eps = 1e-4
    nx, ny, nz = max(x, eps), max(y, eps), max(z, eps)
    if (nx, ny, nz) != (x, y, z):
        attr.Set(Gf.Vec3f(nx, ny, nz))
        changed = True
    return changed


def set_link_defaults(prim):
    """Set damping, velocity caps, sleep threshold and clamp inertia on link-like prims."""
    ensure_attr(prim, "physics:linearDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physics:angularDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)

    ensure_attr(prim, "physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, 10.0).Set(10.0)
    ensure_attr(prim, "physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float, 1000.0).Set(1000.0)
    ensure_attr(prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)

    clamp_diagonal_inertia(prim)


def is_joint_prim(prim):
    """Heuristic: USD physics joints usually have a Physics*Joint typename."""
    t = prim.GetTypeName()
    return bool(t and t.lower().startswith("physics") and t.lower().endswith("joint"))


def remove_drive_components(prim):
    """Remove drive API instances and drive:* attributes from a joint prim."""
    try:
        for api in list(prim.GetAppliedSchemas()):
            if "DriveAPI" in api:
                instance = api.split(":", 1)[1] if ":" in api else None
                try:
                    prim.RemoveAPI(UsdPhysics.DriveAPI, instance)
                except Exception:
                    pass
    except Exception:
        pass
    for n in [
        "drive:stiffness", "drive:damping", "drive:maxForce",
        "drive:targetVelocity", "drive:targetPosition",
        "drive:angular:stiffness", "drive:angular:damping", "drive:angular:maxForce",
        "drive:linear:stiffness", "drive:linear:damping", "drive:linear:maxForce",
    ]:
        a = prim.GetAttribute(n)
        if a:
            a.Clear()


def set_exclude_from_articulation(prim, value=True):
    """Set both physics:* and physxJoint:* for UI/environment variance."""
    ensure_attr(prim, "physics:excludeFromArticulation", Sdf.ValueTypeNames.Bool, value).Set(value)
    ensure_attr(prim, "physxJoint:excludeFromArticulation", Sdf.ValueTypeNames.Bool, value).Set(value)


def get_or_create_drive_api(prim, instance="angular"):
    try:
        api = UsdPhysics.DriveAPI(prim, instance)
        if not api:
            api = UsdPhysics.DriveAPI.Apply(prim, instance)
        return api
    except Exception:
        return None


def set_joint_drive_params(prim, max_vel_deg_per_sec):
    """Set physxJoint:maxJointVelocity (deg/s) and tune drive stiffness/damping based on maxForce."""
    ensure_attr(prim, "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, float(max_vel_deg_per_sec)).Set(float(max_vel_deg_per_sec))

    # Ensure angular drive instance exists
    api = get_or_create_drive_api(prim, "angular")

    # Read maxForce from common places
    max_force = 0.0
    for key in ("drive:angular:maxForce", "drive:maxForce", "drive:linear:maxForce"):
        a = prim.GetAttribute(key)
        if a:
            v = a.Get()
            if v is not None:
                try:
                    max_force = float(v)
                    break
                except Exception:
                    pass

    # Also write back maxForce to the angular instance so UI sees it there
    ensure_attr(prim, "drive:angular:maxForce", Sdf.ValueTypeNames.Float, max_force).Set(max_force)

    stiffness = 100.0 * max_force
    vmax = max(1e-6, float(max_vel_deg_per_sec))
    damping = 10.0 * stiffness / vmax

    # Prefer DriveAPI attribute writers when available
    try:
        if api:
            api.CreateStiffnessAttr().Set(stiffness)
            api.CreateDampingAttr().Set(damping)
    except Exception:
        pass

    # Also set raw attributes for compatibility
    ensure_attr(prim, "drive:angular:stiffness", Sdf.ValueTypeNames.Float, stiffness).Set(stiffness)
    ensure_attr(prim, "drive:angular:damping", Sdf.ValueTypeNames.Float, damping).Set(damping)
    ensure_attr(prim, "drive:stiffness", Sdf.ValueTypeNames.Float, stiffness).Set(stiffness)
    ensure_attr(prim, "drive:damping", Sdf.ValueTypeNames.Float, damping).Set(damping)


def add_physics_scene(stage):
    """Create /World/PhysicsScene and set requested PhysX scene attributes."""
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    prim = scene.GetPrim()

    ensure_attr(prim, "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, False).Set(False)
    ensure_attr(prim, "physxScene:collisionSystem", Sdf.ValueTypeNames.Token, "PCM").Set("PCM")
    ensure_attr(prim, "physxScene:solverType", Sdf.ValueTypeNames.Token, "PGS").Set("PGS")
    ensure_attr(prim, "physxScene:broadphaseType", Sdf.ValueTypeNames.Token, "GPU").Set("GPU")
    ensure_attr(prim, "physxScene:maxPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(prim, "physxScene:maxVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)
    ensure_attr(prim, "physxScene:minPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(prim, "physxScene:minVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)

    # Ensure physics:timeStepsPerSecond = 200 (Scene-level)
    ensure_attr(prim, "physics:timeStepsPerSecond", Sdf.ValueTypeNames.Float, 200.0).Set(200.0)
    # Some builds treat it as double; author both if needed
    try:
        ensure_attr(prim, "physics:timeStepsPerSecond", Sdf.ValueTypeNames.Double, 200.0).Set(200.0)
    except Exception:
        pass


def main():
    args = parse_args()
    if not PXRC_AVAILABLE:
        print("[ERROR] pxr (USD Python API) not found. Run inside USD/Omniverse/Isaac environment.")
        sys.exit(1)

    stage = Usd.Stage.Open(args.usda)
    if not stage:
        print(f"[ERROR] Open failed: {args.usda}")
        sys.exit(2)

    print("[INFO] Setting stage units (upAxis/meters/kilograms)...")
    set_stage_units(stage)

    # DO NOT change Stage timeCodesPerSecond (keep initial)

    print(f"[INFO] Locating robot prim: {args.robot}")
    robot = stage.GetPrimAtPath(args.robot)
    if not robot or not robot.IsValid():
        for p in stage.Traverse():
            if p.GetName() == args.robot.strip("/"):
                robot = p
                break
    if not robot or not robot.IsValid():
        print(f"[ERROR] Robot prim not found: {args.robot}")
        sys.exit(3)

    print("[INFO] Applying robot-level articulation settings...")
    set_robot_level_attrs(robot)

    # Load joint velocity limits (deg/s) from SDF if provided
    joint_vel_deg = read_sdf_joint_vel_limits(args.sdf) if args.sdf else {}

    print("[INFO] Processing links and joints under the robot hierarchy...")
    robot_path_str = str(robot.GetPath())
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(robot_path_str):
            continue

        t = prim.GetTypeName()

        # Link-like prims
        if t in ("Xform", "PhysicsRigidBody", "Mesh", "Cone", "Cube",
                 "Cylinder", "Sphere", "Capsule", "UsdGeomMesh"):
            set_link_defaults(prim)

        # Joints
        if is_joint_prim(prim):
            name = prim.GetName()
            lower = name.lower()
            if lower.endswith("passive") or lower.endswith("_passive"):
                remove_drive_components(prim)
            elif lower.endswith("loop") or lower.endswith("_loop"):
                remove_drive_components(prim)
                set_exclude_from_articulation(prim, True)
            else:
                # Drive remains: set max joint velocity + stiffness/damping
                v = joint_vel_deg.get(name)
                if v is None:
                    base = name.split(":")[-1]
                    v = joint_vel_deg.get(base)
                if v is not None:
                    set_joint_drive_params(prim, v)

    print("[INFO] Ensuring PhysicsScene exists and is configured...")
    add_physics_scene(stage)

    stage.GetRootLayer().Save()
    print("✅ USDA post-process completed.")


if __name__ == "__main__":
    main()
