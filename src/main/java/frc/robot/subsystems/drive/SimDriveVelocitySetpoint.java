package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

/**
 * Shared closed-loop drive velocity setpoint math for Maple sim ({@link ModuleIOSim} Talon and
 * {@link ModuleIOSimMapleDirect} Java PID). Tuner {@code DriveMotorGains} stay in {@code TunerConstants}; this is the
 * common wheel→motor conversion and per-module scale.
 */
public final class SimDriveVelocitySetpoint {

  private SimDriveVelocitySetpoint() {}

  /**
   * Motor shaft velocity setpoint (rotations/s) for Phoenix {@code Velocity*} closed loop, from commanded wheel rad/s.
   *
   * @param wheelVelocityRadPerSec desired wheel angular velocity (rad/s)
   * @param perModuleSpeedMultiplier see {@link SimDriveSpeedMultipliers}
   * @param driveMotorMotorToWheelRatio Phoenix drive motor rotations per wheel rotation (typically {@code
   *     SwerveModuleConstants#DriveMotorGearRatio})
   */
  public static double motorRotationsPerSecFromWheelRadPerSec(
      double wheelVelocityRadPerSec, double perModuleSpeedMultiplier, double driveMotorMotorToWheelRatio) {
    double scaledWheelRadPerSec = wheelVelocityRadPerSec * perModuleSpeedMultiplier;
    return Units.radiansToRotations(scaledWheelRadPerSec) * driveMotorMotorToWheelRatio;
  }
}
