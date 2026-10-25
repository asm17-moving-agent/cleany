"""Build the original Cleany robot and Facility geometry for simulator adapters."""
import json
import math
from pathlib import Path
import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Sdf
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig


def build_stage(output: Path, config: dict):
    OUTPUT, CONFIG = output, config
    path = URDFImporter().import_urdf(URDFImporterConfig(
        urdf_path=str(OUTPUT / "assets/cleany.urdf"), usd_path=str(OUTPUT / "assets/usd"),
        ros_package_paths=[{"name": "cleany_description", "path": "/main/ros2_ws/src/cleany_description"}],
        fix_base=False, merge_fixed_joints=False, merge_mesh=False,
        collision_from_visuals=False, allow_self_collision=False,
        joint_drive_type="force", joint_target_type="position",
        override_joint_stiffness=1200., override_joint_damping=60.))
    if not omni.usd.get_context().open_stage(path):
        raise RuntimeError("Cannot open imported robot")
    stage = omni.usd.get_context().get_stage()
    stage.GetPrimAtPath("/cleany").GetVariantSet("Physics").SetVariantSelection("physx")
    UsdGeom.Xformable(stage.GetPrimAtPath("/cleany")).AddTranslateOp().Set(Gf.Vec3d(*CONFIG["spawn"]))
    UsdGeom.Xformable(stage.GetPrimAtPath("/cleany")).AddRotateZOp().Set(math.degrees(CONFIG.get("spawn_yaw", 0.)))
    for prim in stage.Traverse():
        if prim.IsInstance():
            prim.SetInstanceable(False)
    base = stage.GetPrimAtPath("/cleany/Geometry/base_link")
    base.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(
        [s for s in base.GetAppliedSchemas() if "Articulation" not in s]))
    UsdPhysics.ArticulationRootAPI.Apply(base)
    physx = PhysxSchema.PhysxArticulationAPI.Apply(base)
    physx.CreateEnabledSelfCollisionsAttr(False)
    cleanup_physics = CONFIG.get("cleanup", {}).get("physics", {})
    physx.CreateSolverPositionIterationCountAttr(cleanup_physics.get("position_iterations", 32))
    physx.CreateSolverVelocityIterationCountAttr(cleanup_physics.get("velocity_iterations", 8))
    motors = json.loads((OUTPUT / "config/motors.json").read_text())
    for prim in stage.Traverse():
        if CONFIG.get("cleanup") and prim.GetName() in (
                "right_gripper_frame", "right_moving_jaw_link"):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr(float(motors[prim.GetName()]["armature"]))
            if CONFIG.get("cleanup"):
                motor = motors[prim.GetName()]
                prim.ApplyAPI("PhysxJointAxisAPI", "angular")
                for name, value in {
                    "armature": float(motor["armature"]),
                    "staticFrictionEffort": float(motor["frictionloss"]),
                    "dynamicFrictionEffort": float(motor["frictionloss"]),
                    # USD angular damping is per degree; motor data is per radian.
                    "viscousFrictionCoefficient": float(motor["damping"])*math.pi/180.,
                }.items():
                    prim.CreateAttribute("physxJointAxis:angular:"+name, Sdf.ValueTypeNames.Float).Set(value)

            if CONFIG.get("cleanup") and prim.GetName() in ("left_gripper_joint", "right_gripper_joint"):
                UsdPhysics.DriveAPI.Get(prim, "angular").CreateMaxForceAttr(.2)
            if prim.GetName() in CONFIG.get("park_joints", {}):
                UsdPhysics.DriveAPI.Get(prim, "angular").CreateTargetPositionAttr(math.degrees(CONFIG["park_joints"][prim.GetName()]))

    def cube(name, center, size):
        prim = UsdGeom.Cube.Define(stage, "/Room/" + name)
        prim.CreateSizeAttr(1.)
        prim.AddTranslateOp().Set(Gf.Vec3d(*center))
        prim.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        return prim

    if CONFIG["world"] == "facility18":
        from world_scene import create_facility
        floor = create_facility(stage, OUTPUT)
    else:
        xmin, ymin, xmax, ymax = CONFIG["bounds"]
        floor = cube("Floor", [(xmin+xmax)/2, (ymin+ymax)/2, -.05], [xmax-xmin, ymax-ymin, .1]).GetPrim()
        for name, x in (("West", xmin), ("East", xmax)):
            cube(name, [x, (ymin+ymax)/2, 1.], [.05, ymax-ymin, 2.])
        for name, y in (("South", ymin), ("North", ymax)):
            cube(name, [(xmin+xmax)/2, y, 1.], [xmax-xmin, .05, 2.])
        desk = CONFIG["desk"]
        cube("Desk", [*desk["center"][:2], .35], [*desk["size"][:2], .7])
        # Physical payload is present but remains untouched in the Action-mock tier.
        payload = cube("Payload", [*desk["center"][:2], .75], [.05, .05, .1])
        UsdPhysics.RigidBodyAPI.Apply(payload.GetPrim())
        UsdPhysics.MassAPI.Apply(payload.GetPrim()).CreateMassAttr(.02)
    # Fixed wheel meshes cannot roll under the ideal planar actuator. Disable
    # ground friction explicitly for this tier; obstacle collisions remain active.
    material = UsdShade.Material.Define(stage, "/Room/IdealBaseGround")
    surface = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    surface.CreateStaticFrictionAttr(0.)
    surface.CreateDynamicFrictionAttr(0.)
    surface.CreateRestitutionAttr(0.)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
    UsdShade.MaterialBindingAPI.Apply(floor).Bind(material, materialPurpose="physics")
    if CONFIG.get("cleanup"):
        from cleanup_assets import create_cleanup_assets
        create_cleanup_assets(stage, CONFIG)
    return stage
