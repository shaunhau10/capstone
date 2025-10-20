from isaacsim import SimulationApp
kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

from pxr import Gf, UsdGeom
import omni.usd
import math
from omni.isaac.core.utils.stage import add_reference_to_stage

USD_PATH = "/home/shaun/capstone/assets/fork.usd" 
PRIM_PATH = "/World/Fork"

# rotation test values (degrees)
YAW   = 0     # rotation around Z
PITCH = 0    # rotation around X
ROLL  = 90     # rotation around Y

# position to spawn
X, Y, Z = 0.0, 0.0, 0.1

stage = omni.usd.get_context().get_stage()

# Create world root if missing
if not stage.GetPrimAtPath("/World").IsValid():
    UsdGeom.Xform.Define(stage, "/World")

# Add reference to fork USD
stage.DefinePrim("/World")
UsdGeom.Xform.Define(stage, "/World/Fork")
stage.GetRootLayer().subLayerPaths

add_reference_to_stage(usd_path=USD_PATH, prim_path=PRIM_PATH)

# Transform
fork_prim = stage.GetPrimAtPath(PRIM_PATH)
xf = UsdGeom.Xformable(fork_prim)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(X, Y, Z))
xf.AddRotateXOp().Set(PITCH)
xf.AddRotateYOp().Set(ROLL)
xf.AddRotateZOp().Set(YAW)

kit.update()

print(f"[INFO] Fork spawned at ({X}, {Y}, {Z}) with rotations:")
print(f"       Pitch (X): {PITCH}°")
print(f"       Roll  (Y): {ROLL}°")
print(f"       Yaw   (Z): {YAW}°")

while kit.is_running():
    kit.update()

kit.close()
