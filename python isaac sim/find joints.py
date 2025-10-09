from pxr import Usd, PhysxSchema, UsdPhysics
import omni.usd

# Optional: if you want Isaac Core helpers
from omni.isaac.core.articulations import Articulation

stage = omni.usd.get_context().get_stage()

# 1) Suche Articulation-Root(s)
articulation_roots = []
for prim in stage.Traverse():
    try:
        if prim.HasAPI(PhysxSchema.PhysxArticulationRootAPI):
            articulation_roots.append(str(prim.GetPath()))
    except Exception:
        pass

if not articulation_roots:
    print("Keine PhysX Articulation Root gefunden. Prüfe, ob USD geladen ist und PhysX aktiviert ist.")
else:
    print("Gefundene Articulation Roots:")
    for p in articulation_roots:
        print(" -", p)

# 2) Falls vorhanden: DoF-Namen via Isaac Core
if articulation_roots:
    prim_path = articulation_roots[0]  # ggf. gewünschten Root wählen
    robot = Articulation(prim_path=prim_path)
    robot.initialize()
    print(f"\nDoF-Anzahl: {robot.num_dof}")
    print("DoF-Namen:")
    for name in robot.dof_names:
        print(name)

# 3) Fallback: reine USD-Inspektion für Joint-Prims mit UsdPhysics-Schemas
print("\nUSD-Physics Joints (falls vorhanden):")
joint_count = 0
for prim in stage.Traverse():
    # Nur unterhalb der ersten Articulation-Root durchsuchen (falls gefunden)
    if articulation_roots and not str(prim.GetPath()).startswith(articulation_roots[0]):
        continue
    for Schema in (UsdPhysics.RevoluteJoint, UsdPhysics.PrismaticJoint,
                   UsdPhysics.SphericalJoint, UsdPhysics.DistanceJoint):
        j = Schema(prim)
        if j:
            joint_count += 1
            print("-", prim.GetPath())
if joint_count == 0:
    print("Keine separaten UsdPhysics-*Joint Prims gefunden (typisch bei PhysX Articulations).")
