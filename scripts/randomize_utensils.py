from isaacsim import SimulationApp
kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import math
import omni.appwindow
import carb
import weakref
import random
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.api import World
import isaacsim.core.utils.stage as stage_utils
from omni.isaac.core.utils.prims import create_prim

Z_OFFSET = 0.02
EDGE_MARGIN = 0.05
SAFE_DISTANCE = 0.10
MAX_ATTEMPTS = 120
SAFE_BUFFER = 0.005

# Scene and asset paths
BASE_SCENE_PATH = "/home/shaun/capstone/scenes/base_scene.usd"
UTENSIL_ASSETS = {"fork":  "/home/shaun/capstone/assets/fork.usd",
                  "spoon": "/home/shaun/capstone/assets/spoon.usd",
                  "plate": "/home/shaun/capstone/assets/plate.usd",
                  "bowl":  "/home/shaun/capstone/assets/bowl.usd",}

def compute_world_bbox(world, prim_path, return_center=False):
    # Compute bounding box and center
    # Returns center, half_x, half_y, top_z if return_center is True
    # else returns min_pt, max_pt
    stage = world.stage
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(0, ["default"])
    bbox = bbox_cache.ComputeWorldBound(prim)
    ext = bbox.ComputeAlignedRange()
    min_pt, max_pt = ext.GetMin(), ext.GetMax()

    if not return_center:
        return min_pt, max_pt

    size = ext.GetSize()
    center = [(min_pt[i] + max_pt[i]) / 2 for i in range(3)]
    return center, size[0] / 2, size[1] / 2, max_pt[2]


def collides(world, placed_bboxes, new_min, new_max, buffer=0.005):
    # Check if 2 bounding box intersects with buffer
    for (pmin, pmax) in placed_bboxes.values():
        if ((new_min[0] - buffer <= pmax[0] and new_max[0] + buffer >= pmin[0]) and
            (new_min[1] - buffer <= pmax[1] and new_max[1] + buffer >= pmin[1]) and
            (new_min[2] - buffer <= pmax[2] and new_max[2] + buffer >= pmin[2])):
            return True
    return False


def clear_utensils(world):
    # Clear all utensils from scene
    utensils = world.stage.GetPrimAtPath("/World/Utensils")
    if utensils.IsValid():
        world.stage.RemovePrim("/World/Utensils")
        print("[INFO] Removed all utensils.")


def safe_top_z(world, path, foam_top_z, z_offset):
    # Calculate the top Z values of a prim
    prim = world.stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return foam_top_z + z_offset
    _, max_pt = compute_world_bbox(world, path)
    return max_pt[2]

def spawn_utensils_on_foam(world, assets):
    if not world.stage.GetPrimAtPath("/World/Utensils").IsValid():
        create_prim("/World/Utensils", "Xform")

    # Foam bounds
    foam_center, foam_hx, foam_hy, foam_top_z = compute_world_bbox(world, "/World/BaseScene/Environment/Foam", return_center=True)

    placed_bboxes = {}

    # Define spawn area within foam bounds
    x_min = foam_center[0]
    x_max = foam_center[0] + foam_hx - EDGE_MARGIN
    y_min = foam_center[1] - foam_hy + EDGE_MARGIN
    y_max = foam_center[1] + foam_hy - EDGE_MARGIN

    # Spawn plate
    plate_usd = assets["plate"]
    plate_x = random.uniform(x_min, x_max)
    plate_y = random.uniform(y_min, y_max)
    plate_z = foam_top_z + Z_OFFSET
    plate_path = "/World/Utensils/plate"
    stage_utils.add_reference_to_stage(usd_path=plate_usd, prim_path=plate_path)
    prim = world.stage.GetPrimAtPath(plate_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(plate_x, plate_y, plate_z))
    kit.update()
    placed_bboxes["plate"] = compute_world_bbox(world, plate_path)
    plate_min, plate_max = placed_bboxes["plate"]
    plate_cx = (plate_min[0] + plate_max[0]) / 2
    plate_cy = (plate_min[1] + plate_max[1]) / 2

    # Spawn bowl
    bowl_usd = assets["bowl"]
    bowl_path = "/World/Utensils/bowl"
    spawn_mode = random.choice(["on_plate", "on_foam"])
    print(f"[INFO] Bowl spawn mode: {spawn_mode}")

    if spawn_mode == "on_plate":
        bowl_x = random.uniform(plate_x - 0.02, plate_x + 0.02)
        bowl_y = random.uniform(plate_y - 0.02, plate_y + 0.02)
        bowl_z = plate_max[2] + 0.03
    elif spawn_mode == "on_foam":
        print("[INFO] Searching for non-intersecting bowl position...")

        for attempt in range(MAX_ATTEMPTS):
            x_candidate = random.uniform(x_min, x_max)
            y_candidate = random.uniform(y_min, y_max)
            z_candidate = foam_top_z + Z_OFFSET

            dx = x_candidate - plate_cx
            dy = y_candidate - plate_cy
            if (dx**2 + dy**2)**0.5 < SAFE_DISTANCE:
                continue

            stage_utils.add_reference_to_stage(usd_path=bowl_usd, prim_path=bowl_path)
            prim_tmp = world.stage.GetPrimAtPath(bowl_path)
            xf = UsdGeom.Xformable(prim_tmp)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(x_candidate, y_candidate, z_candidate))
            kit.update()
            bmin, bmax = compute_world_bbox(world, bowl_path)
            overlap = collides(world, placed_bboxes, bmin, bmax, SAFE_BUFFER)
            world.stage.RemovePrim(bowl_path)

            if not overlap:
                bowl_x, bowl_y, bowl_z = x_candidate, y_candidate, z_candidate
                break

    stage_utils.add_reference_to_stage(usd_path=bowl_usd, prim_path=bowl_path)
    prim = world.stage.GetPrimAtPath(bowl_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(bowl_x, bowl_y, bowl_z))
    kit.update()
    placed_bboxes["bowl"] = compute_world_bbox(world, bowl_path)
    bowl_min, bowl_max = placed_bboxes["bowl"]

    # Spawn spoon and fork
    for name in ["spoon", "fork"]:
        usd = assets[name]
        placed = False

        if spawn_mode == "on_plate":
            possible_targets = ["foam", "bowl"]
        elif spawn_mode == "on_foam":
            possible_targets = ["foam", "plate", "bowl"]

        for _ in range(MAX_ATTEMPTS):
            spawn_on = random.choice(possible_targets)
            yaw = random.uniform(-180, 180)
            pitch = 0
            roll = 0

            if spawn_on == "foam":
                x = random.uniform(foam_center[0], foam_center[0] + foam_hx - EDGE_MARGIN)
                y = random.uniform(foam_center[1] - foam_hy + EDGE_MARGIN, foam_center[1] + foam_hy - EDGE_MARGIN)
                z = foam_top_z + Z_OFFSET

            elif spawn_on == "plate":
                plate_rx = (plate_max[0] - plate_min[0]) / 2
                plate_ry = (plate_max[1] - plate_min[1]) / 2

                theta = random.uniform(0, 2 * math.pi)
                offset_ratio = 0.85
                x = plate_cx + math.cos(theta) * plate_rx * offset_ratio
                y = plate_cy + math.sin(theta) * plate_ry * offset_ratio
                yaw = math.degrees(theta + math.pi)
                z = plate_max[2] + 0.1 

                pitch = random.uniform(-2, 2)
                roll = random.uniform(-2, 2)

            elif spawn_on == "bowl":
                bowl_cx = (bowl_min[0] + bowl_max[0]) / 2
                bowl_cy = (bowl_min[1] + bowl_max[1]) / 2
                bowl_rx = (bowl_max[0] - bowl_min[0]) / 2
                bowl_ry = (bowl_max[1] - bowl_min[1]) / 2

                theta = random.uniform(0, 2 * math.pi)
                offset_ratio = 0.85
                x = bowl_cx + math.cos(theta) * bowl_rx * offset_ratio
                y = bowl_cy + math.sin(theta) * bowl_ry * offset_ratio
                z = bowl_max[2] + 0.1
                yaw = math.degrees(theta + math.pi)
                roll = 70
                pitch = 0

            prim_path = f"/World/Utensils/{name}"
            stage_utils.add_reference_to_stage(usd_path=usd, prim_path=prim_path)
            prim = world.stage.GetPrimAtPath(prim_path)
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            xf.AddRotateZOp().Set(yaw)
            xf.AddRotateYOp().Set(roll)
            xf.AddRotateXOp().Set(pitch)
            kit.update()

            bmin, bmax = compute_world_bbox(world, prim_path)
            overlap = collides(world, placed_bboxes, bmin, bmax, SAFE_BUFFER)

            if not overlap:
                placed_bboxes[name] = (bmin, bmax)
                print(f"[INFO] {name.capitalize()} spawned on {spawn_on} at ({x:.3f}, {y:.3f}, {z:.3f}) tilt ({pitch:.1f}, {roll:.1f}).")
                placed = True
                break
            else:
                world.stage.RemovePrim(prim_path)

        if not placed:
            print(f"[WARN] Could not safely place {name} after {MAX_ATTEMPTS} attempts.")

class KeyboardHandler:
    def __init__(self, world):
        self.world = world
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(event, *args),
        )
        self.running = True
        print("[INFO] Keyboard ready — press 'r' to respawn utensils, 'ESC' to quit.")

    def __del__(self):
        if self._keyboard_sub:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)

    def _on_keyboard_event(self, event, *args):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            key = event.input.name
            if key in ("R", "r"):
                print("[INFO] Respawning utensils...")
                self.world.stop()
                kit.update()
                clear_utensils(self.world)
                spawn_utensils_on_foam(self.world, UTENSIL_ASSETS)
                kit.update()
                self.world.reset()
                self.world.play()
                print("[INFO] Utensils respawned.")
            elif key == "ESCAPE":
                print("[INFO] Exiting simulation.")
                self.running = False
        return True

world = World(stage_units_in_meters=1.0)
stage_utils.add_reference_to_stage(BASE_SCENE_PATH, "/World/BaseScene")
clear_utensils(world)
spawn_utensils_on_foam(world, UTENSIL_ASSETS)
world.stop()
kit.update()
world.reset()
kit.update()
world.play()

keyboard = KeyboardHandler(world)
while kit.is_running() and keyboard.running:
    world.step(render=True)
    kit.update()

del keyboard
kit.close()
