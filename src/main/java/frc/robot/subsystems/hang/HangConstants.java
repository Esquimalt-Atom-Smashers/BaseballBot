package frc.robot.subsystems.hang;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/** Constants for the Hang (one motor, position-controlled) subsystem. */
public final class HangConstants {

  private HangConstants() {}

  /** CAN ID of the Hang motor (NEO 550 on SPARK MAX). */
  public static final int kMotorId = 58;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kBrake;

  /** Set true if positive output moves the Hang the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 45;

  /** Linear travel (meters) per motor rotation. */
  public static final double kMetersPerRotation = 0.01;

  /** Hang meters per motor rotation (output / input). */
  public static final double kGearRatio = 5.0;

  /** PID gains for onboard position control. */
  public static final double kP = 24.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Period for sending signals to the motor. */
  public static final int kSignalsPeriodMs = 31;
  public static final int kEncoderVelocitySignalPeriodMs = 31;

  /** Target position when the Hang is in Stored (retracted) mode. */
  public static final double kStoredPositionMeters = Units.inchesToMeters(0.0);
  
  /** Target position when the Hang is in Hanging (retracted part way) mode. */
  public static final double kHangingPositionMeters = Units.inchesToMeters(2.0);

  /** Target position when the Hang is in Level_1 (extended) mode. */
  public static final double kLevel1PositionMeters = Units.inchesToMeters(7.75);

  /** Minimum extension (fully retracted). */
  public static final double kMinMeters = Units.inchesToMeters(0.0);

  /** Maximum extension (fully extended). */
  public static final double kMaxMeters = Units.inchesToMeters(8.5);

  /** Tolerance for considering the Hang at target (measured vs target). */
  public static final double kAtTargetToleranceMeters = Units.inchesToMeters(0.10);

  /** Delta Meter per step. */
  public static final double kStepMeters = Units.inchesToMeters(0.2);

  /** SIM-only position ramp rate used by {@link HangIOSim} (in/s). */
  public static final double kSimPositionRampRateInchesPerSec = 6.931;
}
