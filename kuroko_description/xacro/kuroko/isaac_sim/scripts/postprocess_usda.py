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
    ap.add_argument("--usda", required=True)
    ap.add_argument("--sdf", required=False)
    ap.add_argument("--robot", default="/kuroko")
    return ap.parse_args()


def read_sdf_joint_vel_limits(sdf_path):
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
                            if child.tag.endswith("velocity"):
                                try:
                                    vel = float(child.text.strip())
                                except Exception:
                                    pass
                if vel is not None:
                    limits[name] = vel * 180.0 / 3.1417
    except Exception as e:
        print(f"[WARN] SDF parse failed: {e}")
    return limits


def ensure_attr(prim, name, type_name, default_value=None):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    if default_value is not None and not attr.HasAuthoredValueOpinion():
        attr.Set(default_value)
    return attr


def set_stage_units(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    try:
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    except Exception:
        pass


def set_stage_timecodes(stage, fps=200.0):
    # いくつかのUIは PhysicsScene の値でなく stage の timeCodesPerSecond を見る場合がある
    try:
        stage.SetTimeCodesPerSecond(fps)
    except Exception:
        pass


def set_robot_level_attrs(robot_prim):
    # すでに OK だった solver iteration はそのまま
    ensure_attr(robot_prim, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 32).Set(32)
    ensure_attr(robot_prim, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1).Set(1)

    # Self Collision はアーティキュレーション側に明示
    ensure_attr(robot_prim, "physxArticulation:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, False).Set(False)

    # 念のためロボット直下にも sleep / stabilization を書く（UI差対策）
    ensure_attr(robot_prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)
    ensure_attr(robot_prim, "physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float, 0.0001).Set(0.0001)


def clamp_diagonal_inertia(prim):
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
    # 標準属性 + PhysX拡張の両方に設定（UI/環境差異対策）
    ensure_attr(prim, "physics:linearDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physics:angularDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)
    ensure_attr(prim, "physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float, 0.01).Set(0.01)

    ensure_attr(prim, "physxRigidBody:maxLinearVelocity", Sdf.ValueTypeNames.Float, 10.0).Set(10.0)
    ensure_attr(prim, "physxRigidBody:maxAngularVelocity", Sdf.ValueTypeNames.Float, 1000.0).Set(1000.0)
    ensure_attr(prim, "physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float, 0.001).Set(0.001)

    clamp_diagonal_inertia(prim)


def is_joint_prim(prim):
    t = prim.GetTypeName()
    return bool(t and t.lower().startswith("physics") and t.lower().endswith("joint"))


def remove_drive_components(prim):
    # APIインスタンス付き・無しの両方を除去
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
    # 残存属性もクリア
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
    # Max Joint Velocity
    ensure_attr(prim, "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float, float(max_vel_deg_per_sec)).Set(float(max_vel_deg_per_sec))

    # Drive（Angular優先）
    api = get_or_create_drive_api(prim, "angular")
    # maxForce は既存値を読む。なければ 0
    max_force = 0.0
    for key in ("drive:angular:maxForce", "drive:maxForce"):
        a = prim.GetAttribute(key)
        if a:
            v = a.Get()
            if v is not None:
                try:
                    max_force = float(v)
                    break
                except Exception:
                    pass
    # 必要なら angular 側に maxForce を作成
    ensure_attr(prim, "drive:angular:maxForce", Sdf.ValueTypeNames.Float, max_force).Set(max_force)

    stiffness = 100.0 * max_force
    vmax = max(1e-6, float(max_vel_deg_per_sec))
    damping = 10.0 * stiffness / vmax

    # angular インスタンスに値を入れる（UIで Drive->Angular に表示させる）
    ensure_attr(prim, "drive:angular:stiffness", Sdf.ValueTypeNames.Float, stiffness).Set(stiffness)
    ensure_attr(prim, "drive:angular:damping", Sdf.ValueTypeNames.Float, damping).Set(damping)

    # 互換のため非インスタンス属性も更新
    ensure_attr(prim, "drive:stiffness", Sdf.ValueTypeNames.Float, stiffness).Set(stiffness)
    ensure_attr(prim, "drive:damping", Sdf.ValueTypeNames.Float, damping).Set(damping)


def add_physics_scene(stage):
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
    ensure_attr(prim, "physics:timeStepsPerSecond", Sdf.ValueTypeNames.Float, 200.0).Set(200.0)


def main():
    args = parse_args()
    if not PXRC_AVAILABLE:
        print("[ERROR] pxr (USD Python API) が見つかりません。USD/Omniverse 環境で実行してください。")
        sys.exit(1)

    stage = Usd.Stage.Open(args.usda)
    if not stage:
        print(f"[ERROR] Open failed: {args.usda}")
        sys.exit(2)

    set_stage_units(stage)
    set_stage_timecodes(stage, 200.0)

    robot = stage.GetPrimAtPath(args.robot)
    if not robot or not robot.IsValid():
        # 名前検索（最後の要素一致）
        for p in stage.Traverse():
            if p.GetName() == args.robot.strip("/"):
                robot = p
                break
    if not robot or not robot.IsValid():
        print(f"[ERROR] Robot prim not found: {args.robot}")
        sys.exit(3)

    set_robot_level_attrs(robot)

    # SDF 速度（deg/s）読み込み
    joint_vel_deg = read_sdf_joint_vel_limits(args.sdf) if args.sdf else {}

    # Traverse robot subtree
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(str(robot.GetPath())):
            continue

        t = prim.GetTypeName()

        # Link相当（RigidBody API が乗るものを幅広くカバー）
        if t in ("Xform", "PhysicsRigidBody", "Mesh", "Cone", "Cube", "Cylinder", "Sphere", "Capsule", "UsdGeomMesh"):
            set_link_defaults(prim)

        # Joint
        if is_joint_prim(prim):
            name = prim.GetName()
            if name.endswith("passive") or name.endswith("_passive"):
                remove_drive_components(prim)
            elif name.endswith("loop") or name.endswith("_loop"):
                remove_drive_components(prim)
                set_exclude_from_articulation(prim, True)
            else:
                # Drive残し：速度と剛性・減衰を設定
                v = joint_vel_deg.get(name)
                if v is None:
                    base = name.split(":")[-1]
                    v = joint_vel_deg.get(base)
                if v is not None:
                    set_joint_drive_params(prim, v)

    add_physics_scene(stage)

    stage.GetRootLayer().Save()
    print("✅ USDA post-process completed.")


if __name__ == "__main__":
    main()
