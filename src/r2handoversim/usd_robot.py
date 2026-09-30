"""Kinematic replay using an existing UR5e/Robotiq USD's meshes and joint frames."""
import numpy as np


class UsdRobot:
    def __init__(self, stage, usd_path, translation):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        self.stage, self.Gf, self.UsdGeom = stage, Gf, UsdGeom
        self.root = stage.DefinePrim('/World/AssetRobot', 'Xform')
        self.root.GetReferences().AddReference(str(usd_path))
        xform=UsdGeom.Xformable(self.root);xform.ClearXformOpOrder()
        xform.AddTranslateOp(opSuffix='placement').Set(Gf.Vec3d(*translation))
        for _ in range(3):
            for prim in list(Usd.PrimRange(self.root)):
                if prim.IsInstance(): prim.SetInstanceable(False)
        self.joints = []
        for prim in list(Usd.PrimRange(self.root)):
            if prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                parent, child = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
                if parent and child:
                    def frame(index):
                        pos = getattr(joint, f'GetLocalPos{index}Attr')().Get()
                        rot = getattr(joint, f'GetLocalRot{index}Attr')().Get()
                        m = Gf.Matrix4d(1); m.SetRotate(Gf.Quatd(rot)); m.SetTranslateOnly(Gf.Vec3d(pos))
                        return np.array(m).T
                    self.joints.append((prim.GetName(), parent[0], child[0], frame(0), frame(1),
                                        prim.GetAttribute('physics:axis').Get() or 'Z'))
                prim.SetActive(False)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI): prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        self.ops = {}
        for _, _, child, *_ in self.joints:
            p = stage.GetPrimAtPath(child)
            x = UsdGeom.Xformable(p); x.ClearXformOpOrder()
            self.ops[str(child)] = x.AddTransformOp(opSuffix='replay')
        self.colliders = [p.GetPath() for p in Usd.PrimRange(self.root)
                          if p.HasAPI(UsdPhysics.CollisionAPI) and '/world/' not in str(p.GetPath())]
        self.mesh_count = sum(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(self.root))
        if not self.mesh_count or len(self.joints) < 6:
            raise ValueError('Robot USD has no accessible meshes or six arm joints')

    def world_matrix(self, path):
        return np.array(self.UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T

    def update(self, q, opening):
        names = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
        closure = float(np.clip((.085-opening)/.085, 0, 1)*.80285)
        for name, parent, child, f0, f1, axis in self.joints:
            angle = next((float(q[i]) for i, token in enumerate(names) if token+'_joint' in name), None)
            if angle is None: angle = -closure if 'finger_tip' in name else closure
            r = self.Gf.Matrix4d(1)
            r.SetRotate(self.Gf.Rotation(self.Gf.Vec3d(*{'X':(1,0,0),'Y':(0,1,0),'Z':(0,0,1)}[axis]), np.degrees(angle)))
            world = self.world_matrix(parent) @ f0 @ np.array(r).T @ np.linalg.inv(f1)
            actual_parent = self.stage.GetPrimAtPath(child).GetParent().GetPath()
            local = np.linalg.inv(self.world_matrix(actual_parent)) @ world
            self.ops[str(child)].Set(self.Gf.Matrix4d(*local.T.reshape(-1).tolist()))

    def tool_pose(self):
        """Logical tool frame: between pad tips; Y closes, +Z approaches."""
        from pxr import Usd, UsdGeom
        tips=[]
        for side in ['left','right']:
            prim=self.stage.GetPrimAtPath(f'/World/AssetRobot/robotiq_85_{side}_finger_tip_link')
            bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render']).ComputeUntransformedBound(prim)
            local=np.r_[np.array(bbox.ComputeAlignedRange().GetMidpoint()),1.]
            tips.append((self.world_matrix(prim.GetPath())@local)[:3])
        wrist=self.world_matrix('/World/AssetRobot/ur5e_wrist_3_link')
        center=np.mean(tips,axis=0)
        y=tips[0]-tips[1];y/=np.linalg.norm(y)
        z=wrist[:3,2];z=z-y*np.dot(y,z);z/=np.linalg.norm(z)
        pose=np.eye(4);pose[:3,:3]=np.column_stack([np.cross(y,z),y,z]);pose[:3,3]=center+.015*z
        return pose
