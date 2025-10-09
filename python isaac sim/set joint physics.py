# offline_update_go2w_drives.py
from pxr import Usd, UsdPhysics, PhysxSchema

USD_PATH = "/home/bauya2/unitreerobotics/unitree_mujoco/unitree_robots/go2w/go2w/go2w_adjusted.usd"

GROUP_VALUES = {
    "hip_thigh": dict(maxVel=1724.60303, armature=0.0, stiff=1.0e7,  damp=1.0e5,  maxForce=23.7),
    "calf":      dict(maxVel=899.5437,   armature=0.0, stiff=1.0e7,  damp=1.0e5,  maxForce=45.43),
    "wheel":     dict(maxVel=1724.60303, armature=0.0, stiff=0.0,    damp=1.0e5,  maxForce=0.0),
}

def pick_group(name: str):
    l = name.lower()
    if "wheel" in l: return "wheel"
    if "calf"  in l: return "calf"
    if "hip" in l or "thigh" in l: return "hip_thigh"
    return None

def ensure_attr(attr, create_fn): return attr if attr else create_fn()

def set_physx_joint_attrs(prim, max_vel, armature):
    japi = PhysxSchema.PhysxJointAPI.Apply(prim)
    ensure_attr(japi.GetMaxJointVelocityAttr(), japi.CreateMaxJointVelocityAttr).Set(max_vel)
    ensure_attr(japi.GetArmatureAttr(),        japi.CreateArmatureAttr).Set(armature)

def set_drive_attrs(prim, is_revolute, stiff, damp, max_force):
    if is_revolute:
        dapi = UsdPhysics.DriveAPI(prim, "angular") or UsdPhysics.DriveAPI.Apply(prim, "angular")
        ensure_attr(dapi.GetStiffnessAttr(), dapi.CreateStiffnessAttr).Set(stiff)
        ensure_attr(dapi.GetDampingAttr(),   dapi.CreateDampingAttr).Set(damp)
        ensure_attr(dapi.GetMaxForceAttr(),  dapi.CreateMaxForceAttr).Set(max_force)
    else:
        used = False
        for axis in ("x","y","z"):
            d = UsdPhysics.DriveAPI(prim, axis)
            if d:
                ensure_attr(d.GetStiffnessAttr(), d.CreateStiffnessAttr).Set(stiff)
                ensure_attr(d.GetDampingAttr(),   d.CreateDampingAttr).Set(damp)
                ensure_attr(d.GetMaxForceAttr(),  d.CreateMaxForceAttr).Set(max_force)
                used = True
        if not used:
            d = UsdPhysics.DriveAPI.Apply(prim, "x")
            ensure_attr(d.GetStiffnessAttr(), d.CreateStiffnessAttr).Set(stiff)
            ensure_attr(d.GetDampingAttr(),   d.CreateDampingAttr).Set(damp)
            ensure_attr(d.GetMaxForceAttr(),  d.CreateMaxForceAttr).Set(max_force)

def main():
    stage = Usd.Stage.Open(USD_PATH)
    counts = {"hip_thigh":0,"calf":0,"wheel":0,"skipped":0}
    for prim in stage.Traverse():
        rev = UsdPhysics.RevoluteJoint(prim)
        pri = UsdPhysics.PrismaticJoint(prim)
        sph = UsdPhysics.SphericalJoint(prim)
        dis = UsdPhysics.DistanceJoint(prim)
        if not (rev or pri or sph or dis):
            continue
        grp = pick_group(prim.GetName())
        if not grp:
            counts["skipped"] += 1
            continue
        v = GROUP_VALUES[grp]
        set_physx_joint_attrs(prim, v["maxVel"], v["armature"])
        set_drive_attrs(prim, is_revolute=bool(rev), stiff=v["stiff"], damp=v["damp"], max_force=v["maxForce"])
        counts[grp] += 1
    stage.GetRootLayer().Save()
    print("Updated joints:", counts)
    print("Saved:", USD_PATH)

if __name__ == "__main__":
    main()
