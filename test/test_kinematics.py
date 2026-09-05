import numpy as np
# استيراد الكلاسات من الموديول بتاعك
from mia_robotics_kinematics.Robot_Kinematics.mecanum import MecanumKinematics


def test_mecanum():
    print("==========================================")
    print("      Testing MecanumKinematics           ")
    print("==========================================")


    L = 0.20
    W = 0.15
    R = 0.05

    mecanum = MecanumKinematics(L=L, W=W, R=R)

    target_Vx = 0.5
    target_Vy = -0.2
    target_Wz = 0.1

    print(
        f"Input Velocities  -> Vx: {target_Vx} m/s | Vy: {target_Vy} m/s | Wz: {target_Wz} rad/s"
    )


    wheel_speeds = mecanum.inverse(target_Vx, target_Vy, target_Wz)
    print("\nCalculated Wheel Angular Velocities (rad/s):")
    print(
        f"  w1 (FL): {wheel_speeds[0]:.3f} | w2 (FR): {wheel_speeds[1]:.3f}"
    )
    print(
        f"  w3 (BL): {wheel_speeds[2]:.3f} | w4 (BR): {wheel_speeds[3]:.3f}"
    )

    calc_Vx, calc_Vy, calc_Wz = mecanum.forward(wheel_speeds)
    print("\nReconstructed Velocities (Forward Kinematics):")
    print(
        f"  Vx: {calc_Vx:.3f} m/s | Vy: {calc_Vy:.3f} m/s | Wz: {calc_Wz:.3f} rad/s"
    )

    is_correct = (
        np.isclose(target_Vx, calc_Vx)
        and np.isclose(target_Vy, calc_Vy)
        and np.isclose(target_Wz, calc_Wz)
    )

    print("------------------------------------------")
    if is_correct:
        print("✅ TEST PASSED: Inverse and Forward match perfectly!")
    else:
        print("❌ TEST FAILED: Mismatch in kinematics conversion.")
    print("------------------------------------------\n")


if __name__ == "__main__":
    test_mecanum()