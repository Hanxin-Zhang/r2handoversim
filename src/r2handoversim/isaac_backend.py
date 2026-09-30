"""Isaac Sim 5.0 standalone replay and PhysX hand-overlap evaluation.

Imports of Kit/USD happen only after SimulationApp starts. Scenes support bundled primitives and explicitly configured local USD/OBJ assets.
"""
import time
import json
from pathlib import Path
import numpy as np
from .evaluation import evaluate, robot_geometry, validate_trial, trial_tcp
from .geometry import box, box_pose, inverse, moved, transform, projected_width, points
from .robot import tcp
from .results import save_results


def replay(trials, output, headless=False, hold=False, render_every=1, screenshot=False, animation=False, run_id=None, hand_collision="boxes",
           video=False, video_speed=1., camera="overview"):
    if render_every < 1:
        raise ValueError("render_every must be at least 1")
    for trial in trials:
        validate_trial(trial)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    results = []
    def save_run(status, error=None):
        (output / "run.json").write_text(json.dumps({"status": status, "run_id": run_id,
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
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, PhysicsSchemaTools
        from omni.physx import get_physx_scene_query_interface
        import carb
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        stage = world.stage
        world.scene.add_default_ground_plane()
        dome = UsdLux.DomeLight.Define(stage, "/World/Light")
        dome.CreateIntensityAttr(1500.)
        set_camera_view(eye=np.array([1.3, -1.8, 1.8]), target=np.array([-.35, -.1, 1.05]))

        def capture_file(path):
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            import asyncio
            path = Path(path).resolve()
            path.unlink(missing_ok=True)
            capture = capture_viewport_to_file(get_active_viewport(), str(path))
            done = asyncio.ensure_future(capture.wait_for_result())
            deadline = time.monotonic() + 60
            while not done.done():
                app.update()
                if time.monotonic() > deadline:
                    done.cancel()
                    raise RuntimeError("Viewport capture timed out after 60 seconds")
            done.result()
            from .video import complete_png
            while not complete_png(path):
                app.update()
                if time.monotonic() > deadline:
                    raise RuntimeError(f"Viewport PNG write timed out: {path}")

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
            from copy import deepcopy
            trial = deepcopy(trial)
            world.stop()
            if stage.GetPrimAtPath("/World/Trial"):
                stage.RemovePrim("/World/Trial")
            UsdGeom.Xform.Define(stage, "/World/Trial")
            for group in ("Robot", "Object", "Hand"):
                UsdGeom.Xform.Define(stage, f"/World/Trial/{group}")
            if stage.GetPrimAtPath('/World/AssetRobot'): stage.RemovePrim('/World/AssetRobot')
            asset_robot = None
            if 'asset_robot' in trial:
                from .usd_robot import UsdRobot
                config = trial['asset_robot']
                asset_robot = UsdRobot(stage, config['usd'], config['translation'])
                original_grasp = transform(trial['T_object_gripper'])
                intentional_width_failure = trial.get('replay_reference', {}).get('assigned_outcome') == 'stability'
                if not intentional_width_failure:
                    from .grasp_fit import fit_grasp
                    fitted, contact_fit = fit_grasp(trial['object_mesh_object'], original_grasp, trial['max_opening_m'])
                    trial['T_object_gripper'] = fitted.tolist()
                    opening, pads = asset_robot.calibrate_opening(trial['planned_joints'][-1], contact_fit['width_m'])
                    # Verify both contacts against the measured USD inner pad faces.
                    contact_fit['measured_pad_geometry'] = pads
                    contact_fit['linkage_command_m'] = opening
                    errors = asset_robot.contact_distances(contact_fit['contact_points_tool'])
                    contact_fit['bilateral_distance_m'] = errors
                    if max(errors) > .0002:
                        raise ValueError(f"{trial['object_id']}: object contacts miss the original finger pad surfaces by {errors} m")
                    trial['asset_contact_fit'] = contact_fit
                else:
                    opening = .085
                    asset_robot.update(trial['planned_joints'][-1], opening)
                    trial['asset_contact_fit'] = {'status':'intentional_width_failure',
                        'scope':'No valid grasp; this reference failure is not corrected into a success'}
                old_goal = tcp(trial['planned_joints'][-1])
                tool_offset = inverse(old_goal) @ asset_robot.tool_pose()
                old_object_pose = old_goal @ inverse(original_grasp)
                new_object_pose = old_goal @ tool_offset @ inverse(trial['T_object_gripper'])
                shift = new_object_pose @ inverse(old_object_pose)
                trial['T_tcp_asset_tool'] = tool_offset.tolist()
                trial['target_T_world_gripper'] = (transform(trial['target_T_world_gripper'])@tool_offset).tolist()
                trial['hand_boxes_world'] = [moved(b, shift) for b in trial['hand_boxes_world']]
                trial['palm_position_world'] = points(shift, trial['palm_position_world']).tolist()
                trial['palm_normal_world'] = (shift[:3,:3]@trial['palm_normal_world']).tolist()
                if 'hand_mesh_world' in trial:
                    trial['hand_mesh_world']['vertices'] = points(shift, trial['hand_mesh_world']['vertices']).tolist()
                # Existing planner checks describe the old proxy tool frame.
                if 'planning' in trial:
                    trial['pre_asset_planning'] = trial.pop('planning')
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Table')).MakeInvisible()
            else:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Table')).MakeVisible()
            if camera == "handover":
                target = .8*trial_tcp(trial, trial["planned_joints"][-1])[:3,3] + .2*np.asarray(trial["palm_position_world"])
                set_camera_view(eye=target+np.array([.48, -.65, .38]), target=target)
            frame_directory = output / f"{trial['id']}_frames"
            if video:
                # A fresh directory prevents stale frames entering a rerun.
                import shutil
                shutil.rmtree(frame_directory, ignore_errors=True)
                frame_directory.mkdir()
            if "delivery" in trial and "keypoints_world" in trial["delivery"]:
                from .robot import segment_box
                delivery = trial["delivery"]
                kp = delivery["keypoints_world"]
                shoulder = delivery["shoulder_midpoint"]
                segments = [(kp["left_shoulder"], kp["right_shoulder"]),
                            (shoulder, delivery["torso_center"]), (shoulder, kp["elbow"]),
                            (kp["elbow"], kp["wrist"])]
                for i, (a, b) in enumerate(segments):
                    draw_box(f"/World/Trial/ReceiverSkeleton/bone_{i}", segment_box(a, b, .009), [.7, .4, .9])
                p = np.asarray(delivery["T_world_object"])[:3, 3]
                draw_box("/World/Trial/ReceiverSkeleton/target_direction",
                         segment_box(p, p+.12*np.asarray(delivery["hand_direction_world"]), .004), [.95, .5, .12])
            for i, b in enumerate(trial.get("obstacle_boxes_world", [])):
                # The default table is already present in the shared stage.
                if b.get("label") != "table":
                    draw_box(f"/World/Trial/Obstacles/part_{i}", b, [.45, .4, .4], collider=True)
            for i, b in enumerate(trial["hand_boxes_world"]):
                draw_box(f"/World/Trial/Hand/part_{i}", b, [1., .64, .31], collider=hand_collision == "boxes")
                if "hand_mesh_world" in trial:
                    UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Trial/Hand/part_{i}")).MakeInvisible()
            if "hand_mesh_world" in trial:
                data = trial["hand_mesh_world"]
                mesh = UsdGeom.Mesh.Define(stage, "/World/Trial/Hand/mesh")
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in data["vertices"]])
                mesh.CreateFaceVertexCountsAttr([3] * len(data["faces"]))
                mesh.CreateFaceVertexIndicesAttr(np.asarray(data["faces"]).reshape(-1).tolist())
                mesh.CreateSubdivisionSchemeAttr("none")
                mesh.CreateDisplayColorAttr([Gf.Vec3f(1., .64, .31)])
                if hand_collision == "mesh":
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
            world.set_simulation_dt(physics_dt=trial["dt_s"], rendering_dt=trial["dt_s"])
            q0 = trial["executed_joints"][0]
            robot_ops = [draw_box(f"/World/Trial/Robot/part_{i}", b, [.35, .6, .9])
                         for i, b in enumerate(robot_geometry(trial, q0))]
            if asset_robot:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Trial/Robot')).MakeInvisible()
            grasp_inverse = inverse(trial["T_object_gripper"])
            object_ops = [draw_box(f"/World/Trial/Object/part_{i}", moved(b, trial_tcp(trial, q0) @ grasp_inverse), [.2, .85, .65])
                          for i, b in enumerate(trial["object_boxes"])]
            point_op = None
            if "object_mesh_object" in trial:
                UsdGeom.Imageable(stage.GetPrimAtPath('/World/Trial/Object')).MakeInvisible()
                data = trial['object_mesh_object']
                mesh = UsdGeom.Mesh.Define(stage, '/World/Trial/ObjectMesh')
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in data['vertices']])
                mesh.CreateFaceVertexCountsAttr([3]*len(data['faces']))
                mesh.CreateFaceVertexIndicesAttr(np.asarray(data['faces']).reshape(-1).tolist())
                mesh.CreateSubdivisionSchemeAttr('none')
                mesh.CreateDisplayColorAttr([Gf.Vec3f(*c) for c in data['colors']])
                mesh.GetDisplayColorPrimvar().SetInterpolation('vertex')
                point_op = UsdGeom.Xformable(mesh.GetPrim()).AddTransformOp()
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
            elif "object_points_object" in trial:
                for i in range(len(object_ops)):
                    UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Trial/Object/part_{i}")).MakeInvisible()
                cloud = UsdGeom.Points.Define(stage, "/World/Trial/ObjectCloud")
                cloud.CreatePointsAttr([Gf.Vec3f(*p) for p in trial["object_points_object"]])
                cloud.CreateWidthsAttr([.0015]*len(trial["object_points_object"]))
                cloud.CreateDisplayColorAttr([Gf.Vec3f(.2, .85, .65)])
                point_op = UsdGeom.Xformable(cloud.GetPrim()).AddTransformOp()
                point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q0)@grasp_inverse).T.reshape(-1).tolist()))
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
            if asset_robot: asset_robot.update(q0, opening)
            world.step(render=True)
            if video or screenshot:
                for _ in range(8):
                    world.render()
            query = get_physx_scene_query_interface()
            contacts = []
            started = time.perf_counter()
            for frame, q in enumerate(trial["executed_joints"]):
                if not app.is_running():
                    raise RuntimeError("Isaac Sim closed before replay finished")
                robot_boxes = robot_geometry(trial, q)
                if asset_robot: asset_robot.update(q, opening)
                for op, b in zip(robot_ops, robot_boxes):
                    update_box(op, b)
                for op, b in zip(object_ops, trial["object_boxes"]):
                    update_box(op, moved(b, trial_tcp(trial,q) @ grasp_inverse))
                if point_op is not None:
                    point_op.Set(Gf.Matrix4d(*(trial_tcp(trial,q)@grasp_inverse).T.reshape(-1).tolist()))
                world.step(render=video or frame % render_every == 0)
                hit_hand = [False]

                def report_hit(hit):
                    if str(hit.collision).startswith("/World/Trial/Hand/"):
                        hit_hand[0] = True
                    return True

                if asset_robot:
                    for path in asset_robot.colliders:
                        a,b=PhysicsSchemaTools.encodeSdfPath(path)
                        query.overlap_shape(a,b,report_hit,False)
                else:
                    for b in robot_boxes:
                        query.overlap_box(carb.Float3(*b["half_extents"]), carb.Float3(*b["center"]),
                                          quaternion(b["rotation"]), report_hit, False)
                contacts.append(hit_hand[0])
                if video:
                    capture_file(frame_directory / f"{frame:06d}.png")
            result = evaluate(trial, contacts)
            result["safe_source"] = f"Isaac Sim PhysX robot-box overlap against static hand {hand_collision} at every frame"
            if asset_robot:
                result['safe_source'] = f'Isaac Sim PhysX actual USD robot collider overlap against hand {hand_collision}'
                result['asset_robot'] = {**trial['asset_robot'], 'mesh_count':asset_robot.mesh_count,
                                        'collider_count':len(asset_robot.colliders), 'T_tcp_asset_tool':trial['T_tcp_asset_tool']}
                result['object_asset'] = {k:trial['object_mesh_object'][k] for k in ('source_path','source_sha256')}
                result['grasp_contact'] = trial['asset_contact_fit']
            result["hand_collision"] = hand_collision
            result["execution_wall_time_s"] = time.perf_counter() - started
            result["backend"] = "isaacsim-physx"
            result["physics_scope"] = "Static hand colliders and " + ("USD robot collider" if asset_robot else "robot-box") + " overlap queries; kinematic robot and rigidly attached object; no grasp dynamics"
            results.append(result)
            # Some Kit installations terminate Python during app.close(). Persist first.
            save_results(results, output)
            save_run("running")
            # Self-contained USD includes procedural shapes and any supplied hand mesh.
            world.stop()
            snapshot = (output / f"{trial['id']}.usda").resolve()
            if not stage.Export(str(snapshot)):
                raise RuntimeError("USD scene export failed")
            result["artifacts"] = {"scene": snapshot.name}
            if video:
                from .video import encode_frames
                destination = output / f"{trial['id']}.mp4"
                encode_frames(frame_directory, destination, len(contacts), trial["dt_s"], video_speed)
                result["artifacts"]["video"] = destination.name
                result["recording"] = {"source": "Isaac Sim viewport", "camera": camera,
                    "captured_frames": len(contacts), "simulation_dt_s": trial["dt_s"],
                    "playback_speed": video_speed, "output_fps": 30, "start_hold_s": 1, "end_hold_s": 2}
                shutil.rmtree(frame_directory)
            if screenshot:
                world.render()
                capture_file(output / f"{trial['id']}.png")
                result["artifacts"]["screenshot"] = f"{trial['id']}.png"
            if animation:
                from .animation import bake
                bake(snapshot, output / f"{trial['id']}_animation.usda", trial, asset_robot=asset_robot, opening=opening if asset_robot else None)
                result["artifacts"]["animation"] = f"{trial['id']}_animation.usda"
            print(f"{trial['id']}: {result['first_failure'] or 'success'} (Isaac Sim)", flush=True)
        save_results(results, output)
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
