package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/** Constants for the Flywheel (one motor, velocity-controlled) subsystem. */
public final class FlywheelConstants {

  private FlywheelConstants() {}

  /** CAN ID of the Flywheel motor. */
  public static final int kMotorId = 56;

  /** Max velocity ramp rate per second (RPM/s). */
  public static final double kVelocityRampRateRpmPerSec = 4000.0;

  /** Neutral mode when the motor is not driven (coast or brake). */
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

  /** Set true if positive velocity spins the Flywheel the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Stator current limit (amps); protects motor and gearbox. */
  public static final double kStatorCurrentLimitAmps = 160.0;

  /** PIDF gains for onboard velocity control and for sim. */
  public static final double kP = 0.4;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kV = 0.117;
  public static final double kS = 0.0;

  /** Flywheel radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 27.0/30.0;

  /** Flywheel radius for converting angular velocity to launch linear velocity (~2 in). */
  public static final double kFlywheelRadiusMeters = 0.0508;

  /** Target velocity in Idle state. */
  public static final double kIdleVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0.0);

  /** Default target velocity. */
  public static final double kDefaultTargetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3200.0);

  /** Minimum Flywheel velocity. */
  public static final double kMinTargetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(0.0);

  /** Maximum Flywheel velocity. */
  public static final double kMaxTargetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(5500.0);

  /** Tolerance for considering the Flywheel at target (measured vs target). */
  public static final double kAtTargetVelocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(150.0);

  /** Delta rad/s per manual step. */
  public static final double kStepRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(100.0);
}
