package frc.robot.subsystems.extender;

import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.util.Units;

/** Constants for the Extender (one motor, position-controlled) subsystem. */
public final class ExtenderConstants {

  private ExtenderConstants() {}

  /** CAN ID of the Extender motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 51;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kBrake;

  /** Set true if positive output moves the Extender the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 40;

  /** Extender radians per motor rotation 1.0 = 1:1 */
  public static final double kGearRatio = 100;

  /** PID gains for onboard position control. */
  public static final double kP = 10.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Period for sending signals to the motor. */
  public static final int kSignalsPeriodMs = 31;
  public static final int kEncoderVelocitySignalPeriodMs = 31;

  /** Target position when the Extender is in Retracted (up) mode. */
  public static final double kUpRad = Units.degreesToRadians(0.0);

  /** Target position when the Extender is in Partial mode. */
  public static final double kPartialRad = Units.degreesToRadians(35.0);

  /** Target position when the Extender is in Extended mode. */
  public static final double kExtendedRad = Units.degreesToRadians(80.0);

  /** Minimum angle (fully retracted). */
  public static final double kMinRad = Units.degreesToRadians(0.0);

  /** Maximum angle (fully extended). */
  public static final double kMaxRad = Units.degreesToRadians(90.0);

  /** Tolerance for considering the Extender at target (measured vs target). */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);

  /** Delta Rad per step. */
  public static final double kStepRadUp = Units.degreesToRadians(10.0);
  public static final double kStepRadDown = Units.degreesToRadians(5.0);
}
