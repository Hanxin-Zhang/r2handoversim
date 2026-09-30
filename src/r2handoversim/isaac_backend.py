"""Isaac Sim 5.0 standalone replay and PhysX hand-overlap evaluation.

Imports of Kit/USD happen only after SimulationApp starts. Scene assets are
created from primitives, avoiding external asset downloads and absolute paths.
"""
import time
import json
from pathlib import Path
import numpy as np
from .evaluation import evaluate, robot_geometry, validate_trial
from .geometry import box, box_pose, inverse, moved, transform
from .robot import tcp
from .results import save_results


def replay(trials, output, headless=False, hold=False, render_every=1, screenshot=False):
    if render_every < 1:
        raise ValueError("render_every must be at least 1")
    for trial in trials:
        validate_trial(trial)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    def save_run(status, error=None):
        (output / "run.json").write_text(json.dumps({"status": status,
            "expected_trials": len(trials), "completed_trials": len(results),
            "error": error}, indent=2))
    save_run("starting")
    try:
        from isaacsim import SimulationApp
    except ImportError as exc:
        raise ImportError("Use Isaac Sim's python.sh, or an environment containing Isaac Sim 5.0. See README.") from exc
    # Do not pass our argparse flags to Kit's argument parser.
    import sys
    saved_argv = sys.argv
    sys.argv = [sys.argv[0]]
    app = SimulationApp({"headless": headless, "width": 1280, "height": 800,
                         "renderer": "RaytracedLighting"})
    sys.argv = saved_argv
    try:
        from isaacsim.core.api import World
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
        from omni.physx import get_physx_scene_query_interface
        import carb
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        stage = world.stage
        world.scene.add_default_ground_plane()
        dome = UsdLux.DomeLight.Define(stage, "/World/Light")
        dome.CreateIntensityAttr(1500.)
        set_camera_view(eye=np.array([1.3, -1.8, 1.8]), target=np.array([-.35, -.1, 1.05]))

        def quaternion(rotation):
            # USD/Gf uses row-vector matrices, NumPy kernel uses columns.
            matrix = Gf.Matrix3d(*np.asarray(rotation).T.reshape(-1).tolist())
            q = matrix.ExtractRotation().GetQuat()
            v = q.GetImaginary()
            return carb.Float4(float(v[0]), float(v[1]), float(v[2]), float(q.GetReal()))

        def draw_box(path, b, color, collider=False):
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(2.)
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            xform = UsdGeom.Xformable(cube.GetPrim())
            xform.ClearXformOpOrder()
            t = box_pose(b)
            matrix = t.copy()
            matrix[:3, :3] = t[:3, :3] @ np.diag(b["half_extents"])
            op = xform.AddTransformOp()
            op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))
            if collider:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            return op

        def update_box(op, b):
            matrix = box_pose(b)
            matrix[:3, :3] = matrix[:3, :3] @ np.diag(b["half_extents"])
            op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()))

        draw_box("/World/Table", box([-.35, 0, .70], [.65, .55, .035]), [.24, .29, .36], collider=True)
        save_run("running")
        for trial in trials:
            world.stop()
            if stage.GetPrimAtPath("/World/Trial"):
                stage.RemovePrim("/World/Trial")
            UsdGeom.Xform.Define(stage, "/World/Trial")
            for i, b in enumerate(trial["hand_boxes_world"]):
                draw_box(f"/World/Trial/Hand/part_{i}", b, [1., .64, .31], collider=True)
            world.set_simulation_dt(physics_dt=trial["dt_s"], rendering_dt=trial["dt_s"])
            q0 = trial["executed_joints"][0]
            robot_ops = [draw_box(f"/World/Trial/Robot/part_{i}", b, [.35, .6, .9])
                         for i, b in enumerate(robot_geometry(trial, q0))]
            grasp_inverse = inverse(trial["T_object_gripper"])
            object_ops = [draw_box(f"/World/Trial/Object/part_{i}", moved(b, tcp(q0) @ grasp_inverse), [.2, .85, .65])
                          for i, b in enumerate(trial["object_boxes"])]
            center = np.asarray(trial["palm_position_world"]) + trial["reach_offset_m"] * np.asarray(trial["palm_normal_world"])/np.linalg.norm(trial["palm_normal_world"])
            for axis in range(3):
                circle = UsdGeom.BasisCurves.Define(stage, f"/World/Trial/ReachRegion/ring_{axis}")
                circle.CreateTypeAttr("linear")
                circle.CreateWrapAttr("periodic")
                circle.CreateCurveVertexCountsAttr([64])
                coordinates = []
                for theta in np.linspace(0, 2*np.pi, 64, endpoint=False):
                    p = center.copy()
                    p[(axis+1)%3] += trial["reach_radius_m"]*np.cos(theta)
                    p[(axis+2)%3] += trial["reach_radius_m"]*np.sin(theta)
                    coordinates.append(Gf.Vec3f(*p.tolist()))
                circle.CreatePointsAttr(coordinates)
                circle.CreateWidthsAttr([.002])
                circle.SetWidthsInterpolation("constant")
                circle.CreateDisplayColorAttr([Gf.Vec3f(.15, .8, .3)])
            world.reset()
            world.step(render=True)
            query = get_physx_scene_query_interface()
            contacts = []
            started = time.perf_counter()
            for frame, q in enumerate(trial["executed_joints"]):
                if not app.is_running():
                    raise RuntimeError("Isaac Sim closed before replay finished")
                robot_boxes = robot_geometry(trial, q)
                for op, b in zip(robot_ops, robot_boxes):
                    update_box(op, b)
                for op, b in zip(object_ops, trial["object_boxes"]):
                    update_box(op, moved(b, tcp(q) @ grasp_inverse))
                world.step(render=frame % render_every == 0)
                hit_hand = [False]

                def report_hit(hit):
                    if str(hit.collision).startswith("/World/Trial/Hand/"):
                        hit_hand[0] = True
                    return True

                for b in robot_boxes:
                    query.overlap_box(carb.Float3(*b["half_extents"]), carb.Float3(*b["center"]),
                                      quaternion(b["rotation"]), report_hit, False)
                contacts.append(hit_hand[0])
            result = evaluate(trial, contacts)
            result["execution_wall_time_s"] = time.perf_counter() - started
            result["backend"] = "isaacsim-physx"
            result["physics_scope"] = "Static hand colliders and robot-box overlap queries; object rigidly replayed; no grasp dynamics"
            results.append(result)
            # Some Kit installations terminate Python during app.close(). Persist first.
            save_results(results, output)
            save_run("running")
            # Self-contained USD contains original primitives only.
            world.stop()
            stage.Export(str((output / f"{trial['id']}.usda").resolve()))
            if screenshot:
                from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
                world.render()
                capture = capture_viewport_to_file(get_active_viewport(), str((output / f"{trial['id']}.png").resolve()))
                # Let the asynchronous capture complete before changing the stage.
                import asyncio
                done = asyncio.ensure_future(capture.wait_for_result())
                while not done.done():
                    app.update()
                done.result()
            print(f"{trial['id']}: {result['first_failure'] or 'success'} (Isaac Sim)", flush=True)
        save_run("succeeded")
        while hold and not headless and app.is_running():
            app.update()
        return results
    except BaseException as exc:
        save_run("failed", str(exc))
        import traceback
        traceback.print_exc()
        raise
    finally:
        app.close()
