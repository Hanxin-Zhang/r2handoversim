"""Bake a self-contained replay USD from an exported final-scene snapshot."""
import numpy as np
from .evaluation import robot_geometry
from .geometry import box_pose, inverse, moved
from .robot import tcp


def bake(snapshot, destination, trial):
    from pxr import Usd, UsdGeom, Gf
    stage = Usd.Stage.Open(str(snapshot))
    stage.SetTimeCodesPerSecond(1/trial["dt_s"])
    stage.SetFramesPerSecond(1/trial["dt_s"])
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(len(trial["executed_joints"])-1)
    grasp_inverse = inverse(trial["T_object_gripper"])
    for frame, q in enumerate(trial["executed_joints"]):
        robot = robot_geometry(trial, q)
        objects = [moved(b, tcp(q) @ grasp_inverse) for b in trial["object_boxes"]]
        for group, boxes in (("Robot", robot), ("Object", objects)):
            for i, b in enumerate(boxes):
                prim = stage.GetPrimAtPath(f"/World/Trial/{group}/part_{i}")
                op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
                matrix = box_pose(b)
                matrix[:3,:3] = matrix[:3,:3] @ np.diag(b["half_extents"])
                op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()), Usd.TimeCode(frame))
    if not stage.Export(str(destination)):
        raise RuntimeError("USD animation export failed")
