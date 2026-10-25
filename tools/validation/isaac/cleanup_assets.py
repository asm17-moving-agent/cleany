"""Physical desk payload and robot-mounted receptacles, independent of perception."""
import sys
import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema


def create_cleanup_assets(stage, config):
    spec = config['cleanup']
    # Reuse the existing procedural object geometry and fixture dimensions.
    sys.path.insert(0, '/main/ros2_ws/src/cleany_mujoco_sim')
    from cleany_mujoco_sim.tabletop_shapes import TabletopShape, cup_vertices, tissue_vertices
    payload = spec['payload']
    size = tuple(payload['collision']['size_m'])
    shape = TabletopShape(payload['visual_type'],payload['geometry'])
    height = size[-1]
    root = UsdGeom.Xform.Define(stage, '/CleanupPayload')
    root.AddTranslateOp().Set(Gf.Vec3d(*spec['payload_position']))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.)
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(payload['mass_kg'])
    material = UsdShade.Material.Define(stage, '/CleanupMaterial')
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(1.5)
    physics.CreateDynamicFrictionAttr(1.5)
    physics.CreateRestitutionAttr(0.)
    # MuJoCo combines equal-priority geom friction with an elementwise maximum.
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr('max')

    def mesh_at(name, vertices, faces, *, visible=True, collision=False):
        mesh = UsdGeom.Mesh.Define(stage, '/CleanupPayload/'+name)
        mesh.CreatePointsAttr([(x,y,z-height/2) for x,y,z in vertices])
        mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([tuple(payload['rgba'][:3])])
        if not visible:
            mesh.MakeInvisible()
        if collision:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim()).CreateContactOffsetAttr(.001)
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material, materialPurpose='physics')
        return mesh

    if shape.kind == 'paper_cup':
        mesh_at('visual',*cup_vertices(size,shape))
        vertices,_ = cup_vertices(size,shape,collision=True)
        count = len(vertices)//4
        faces = ((0,1,3,2),(2,3,5,4),(4,5,7,6),(6,7,1,0),(0,2,4,6),(1,7,5,3))
        for i in range(count):
            panel = [vertices[ring*count+j] for ring in range(4) for j in (i,(i+1)%count)]
            mesh_at(f'wall_{i}',panel,faces,visible=False,collision=True)
        thickness = shape.number('wall_thickness_m')
        base = UsdGeom.Cylinder.Define(stage, '/CleanupPayload/bottom')
        base.CreateRadiusAttr(shape.number('bottom_diameter_m')/2)
        base.CreateHeightAttr(thickness)
        base.AddTranslateOp().Set(Gf.Vec3d(0.,0.,-height/2+thickness/2))
        base.CreateDisplayColorAttr([tuple(payload['rgba'][:3])])
        UsdPhysics.CollisionAPI.Apply(base.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(base.GetPrim()).CreateContactOffsetAttr(.001)
        UsdShade.MaterialBindingAPI.Apply(base.GetPrim()).Bind(material, materialPurpose='physics')
    else:
        mesh_at('tissue',*tissue_vertices(size,shape),collision=True)
    # Receptacles follow the existing rigid base; they have no separate actuator.
    for bin_ in spec['bins']:
        cx, cy = bin_['center_xy_m']
        sx, sy, sz = bin_['outside_size_m']
        thick, z = bin_['wall_thickness_m'], bin_['bottom_z_m']
        parts = [('floor',(cx,cy,z+thick/2),(sx,sy,thick)),
                 ('left',(cx,cy-sy/2+thick/2,z+sz/2),(sx,thick,sz)),
                 ('right',(cx,cy+sy/2-thick/2,z+sz/2),(sx,thick,sz)),
                 ('front',(cx+sx/2-thick/2,cy,z+sz/2),(thick,sy,sz)),
                 ('back',(cx-sx/2+thick/2,cy,z+sz/2),(thick,sy,sz))]
        for name, center, size in parts:
            cube = UsdGeom.Cube.Define(stage, f'/cleany/Geometry/base_link/CleanupBins/{bin_["id"]}/{name}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*center))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            cube.CreateDisplayColorAttr([tuple(bin_['rgba'][:3])])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return root


class PlacementEvidence:
    """Read-only PhysX ground truth, exposed solely to the existing outcome oracle."""
    def __init__(self, physics, node, config):
        from visualization_msgs.msg import MarkerArray
        from payload_geometry import PayloadGeometry
        payload = config['cleanup']['payload']
        self.geometry = PayloadGeometry.from_payload(payload)
        self.body = 'isaac_'+payload['name']
        self.view = physics.create_rigid_body_view('/CleanupPayload')
        self.pub = node.create_publisher(MarkerArray, '/simulation/sorting_ground_truth', 10)
        if self.view.count != 1:
            raise RuntimeError('Missing physical cleanup payload')

    def publish(self, root, seconds):
        from visualization_msgs.msg import Marker, MarkerArray
        from builtin_interfaces.msg import Time
        pose = self.view.get_transforms()[0]
        rotation = Gf.Matrix3d(Gf.Rotation(Gf.Quatd(float(root[6]), Gf.Vec3d(*map(float,root[3:6])))))
        relative = np.asarray(rotation) @ (pose[:3]-root[:3])
        # Bound the actual authored shapes, rather than a rotated enclosing box.
        cup_rotation = np.asarray(Gf.Matrix3d(Gf.Rotation(Gf.Quatd(float(pose[6]), Gf.Vec3d(*map(float,pose[3:6]))))))
        low, high = self.geometry.bounds(relative, np.asarray(rotation) @ cup_rotation.T)
        marker = Marker()
        marker.header.frame_id = 'base_link'
        stamp = round(seconds*1e9)
        marker.header.stamp = Time(sec=stamp//10**9, nanosec=stamp%10**9)
        marker.text, marker.ns = self.body, 'outcome_oracle_only'
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float,(low+high)/2)
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = map(float,high-low)
        self.pub.publish(MarkerArray(markers=[marker]))
