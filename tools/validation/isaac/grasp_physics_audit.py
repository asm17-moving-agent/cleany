"""Read-only simulator diagnostics; never used by perception or robot control."""
import json
import numpy as np
from pxr import UsdGeom, UsdPhysics, PhysxSchema


class GraspPhysicsAudit:
    def __init__(self, stage, physics, output, hz):
        self.dt = 1./hz
        self.bodies = {}
        self.net_contacts = {}
        metadata = {}
        filters = []
        for name in ('right_gripper_frame', 'right_moving_jaw_link'):
            matches = [p for p in stage.Traverse() if p.GetName() == name]
            if len(matches) != 1:
                raise RuntimeError(f'Missing physics audit body: {name}')
            path = str(matches[0].GetPath())
            filters.append(path)
            self.bodies[name] = physics.create_rigid_body_view(path)
            self.net_contacts[name] = physics.create_rigid_contact_view(path, [])
            colliders = []
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith(path+'/') and prim.HasAPI(UsdPhysics.CollisionAPI):
                    collider = UsdPhysics.CollisionAPI(prim)
                    points = UsdGeom.Mesh(prim).GetPointsAttr().Get() if prim.IsA(UsdGeom.Mesh) else None
                    colliders.append(dict(path=str(prim.GetPath()),enabled=collider.GetCollisionEnabledAttr().Get(),
                                          points=0 if points is None else len(points),
                                          contact_offset_m=PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get(),
                                          rest_offset_m=PhysxSchema.PhysxCollisionAPI(prim).GetRestOffsetAttr().Get()))
            metadata[name] = colliders
            if not colliders:
                raise RuntimeError(f'No imported collision geometry for {name}')
        (output/'evaluations/gripper-colliders.json').write_text(json.dumps(metadata,indent=2))
        self.payload = physics.create_rigid_body_view('/CleanupPayload')
        self.robot = physics.create_articulation_view('/cleany/Geometry/base_link')
        solver = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath('/cleany/Geometry/base_link'))
        settings = dict(
            max_force_nm=dict(zip(self.robot.shared_metatype.dof_names,
                                  self.robot.get_dof_max_forces()[0].tolist(), strict=True)),
            armature_kg_m2=dict(zip(self.robot.shared_metatype.dof_names,
                                  self.robot.get_dof_armatures()[0].tolist(), strict=True)),
            stiffness_nm_per_rad=dict(zip(self.robot.shared_metatype.dof_names,
                                  self.robot.get_dof_stiffnesses()[0].tolist(), strict=True)),
            damping_nm_s_per_rad=dict(zip(self.robot.shared_metatype.dof_names,
                                  self.robot.get_dof_dampings()[0].tolist(), strict=True)),
            drive_model_properties=self.robot.get_dof_drive_model_properties().tolist(),
            friction_properties=self.robot.get_dof_friction_properties().tolist(),
            shape_offsets_m={name: dict(contact=view.get_contact_offsets().tolist(),
                                        rest=view.get_rest_offsets().tolist())
                             for name, view in {**self.bodies, 'payload': self.payload}.items()},
            solver_position_iterations=solver.GetSolverPositionIterationCountAttr().Get(),
            solver_velocity_iterations=solver.GetSolverVelocityIterationCountAttr().Get())
        (output/'evaluations/physics-settings.json').write_text(json.dumps(settings, indent=2))
        self.contacts = physics.create_rigid_contact_view('/CleanupPayload', filters)
        self.log = (output/'evaluations/grasp-physics.jsonl').open('w')
        self.render_log = (output/'evaluations/render-physics-delta.jsonl').open('w')

    def snapshot(self):
        return dict(joints=self.robot.get_dof_positions().copy(),
                    velocities=self.robot.get_dof_velocities().copy(),
                    position_targets=self.robot.get_dof_position_targets().copy(),
                    velocity_targets=self.robot.get_dof_velocity_targets().copy(),
                    payload=self.payload.get_transforms().copy(),
                    root=self.robot.get_root_transforms().copy())

    def record_render_delta(self, before, seconds):
        after = self.snapshot()
        delta = {name: float(np.max(np.abs(after[name]-value)))
                 for name, value in before.items()}
        self.render_log.write(json.dumps(dict(simulation_s=seconds, max_abs_delta=delta))+'\n')
        self.render_log.flush()

    def record(self, seconds):
        value = dict(simulation_s=seconds,payload=self.payload.get_transforms().tolist(),
                     joints=dict(zip(self.robot.shared_metatype.dof_names,
                                     self.robot.get_dof_positions()[0].tolist(),strict=True)),
                     bodies={name:view.get_transforms().tolist() for name,view in self.bodies.items()},
                     jaw_contact_force_n=np.asarray(self.contacts.get_contact_force_matrix(self.dt)).tolist())
        value['jaw_net_contact_force_n'] = {
            name: view.get_net_contact_forces(self.dt).tolist()
            for name, view in self.net_contacts.items()}
        self.log.write(json.dumps(value)+'\n')
        self.log.flush()
