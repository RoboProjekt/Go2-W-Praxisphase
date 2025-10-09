# offline_set_go2w_drive_targets.py
# Setzt in der USD die Physics->Drive->Angular TargetPosition (hip/thigh/calf je FR/FL/RR/RL)
# und TargetVelocity (wheel). Anpassung auf Namensschema: "<LEG>_<type>_joint"

from pxr import Usd, UsdPhysics

USD_PATH = "/home/bauya2/unitreerobotics/unitree_mujoco/unitree_robots/go2w/go2w/go2w_adjusted.usd"

# Zielwerte (Radiant / rad/s) – HIER ANPASSEN
TARGET_POS = {
    "FR": {"hip": 0.0, "thigh": 0.0, "calf": 0.0},
    "FL": {"hip": 0.0, "thigh": 0.0, "calf": 0.0},
    "RR": {"hip": 0.0, "thigh": 30.0, "calf": 0.0},
    "RL": {"hip": 0.0, "thigh": 30.0, "calf": 0.0},
}
WHEEL_TARGET_VEL = {"FR": 0.0, "FL": 0.0, "RR": 0.0, "RL": 0.0}  # rad/s

LEG_PREFIXES = ("FR_", "FL_", "RR_", "RL_")
JOINT_TYPES = ("hip", "thigh", "calf", "wheel")

def detect_leg(name: str):
    up = name.upper()
    for leg in ("FR", "FL", "RR", "RL"):
        if up.startswith(leg + "_"):
            return leg
    return None

def detect_joint_type(name: str):
    low = name.lower()
    for jt in JOINT_TYPES:
        if f"_{jt}_" in low or low.endswith(f"_{jt}") or low.startswith(f"{jt}_"):
            return jt
    return None

def ensure_drive(prim, drive_name: str):
    dapi = UsdPhysics.DriveAPI(prim, drive_name)
    if not dapi:
        dapi = UsdPhysics.DriveAPI.Apply(prim, drive_name)
    return dapi

def set_target_position(dapi, value: float):
    (dapi.GetTargetPositionAttr() or dapi.CreateTargetPositionAttr()).Set(float(value))

def set_target_velocity(dapi, value: float):
    (dapi.GetTargetVelocityAttr() or dapi.CreateTargetVelocityAttr()).Set(float(value))

def main():
    stage = Usd.Stage.Open(USD_PATH)
    if stage is None:
        raise RuntimeError(f"Kann USD nicht öffnen: {USD_PATH}")

    counts = {"hip":0, "thigh":0, "calf":0, "wheel":0, "skipped":0}

    for prim in stage.Traverse():
        # Nur physische Gelenke
        if not (UsdPhysics.RevoluteJoint(prim) or UsdPhysics.PrismaticJoint(prim) or UsdPhysics.SphericalJoint(prim)):
            continue

        name = prim.GetName()  # z.B. "FL_hip_joint"
        leg = detect_leg(name)
        jtype = detect_joint_type(name)
        if leg is None or jtype is None:
            counts["skipped"] += 1
            continue

        # Revolute (typisch) → "angular"-Drive
        dapi = ensure_drive(prim, "angular")

        if jtype in ("hip", "thigh", "calf"):
            val = TARGET_POS.get(leg, {}).get(jtype, None)
            if val is not None:
                set_target_position(dapi, val)
                counts[jtype] += 1

        elif jtype == "wheel":
            vel = WHEEL_TARGET_VEL.get(leg, None)
            if vel is not None:
                set_target_velocity(dapi, vel)
                counts["wheel"] += 1

    stage.GetRootLayer().Save()
    print("Targets gesetzt:", counts)
    print("Gespeichert:", USD_PATH)

if __name__ == "__main__":
    main()
