package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the Intake (one motor, voltage controlled) subsystem. */
public final class IntakeConstants {

  private IntakeConstants() {}

  /** CAN ID of the Intake motor. */
  public static final int kMotorId = 50;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Set true if positive voltage spins the Intake the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 60;

  /** Open-loop ramp time from 0 to full output. Limits current spikes on step changes. */
  public static final double kOpenLoopRampRateSec = 0.3;

  /** Period for sending signals to the motor. */
  public static final int kSignalsPeriodMs = 127;
  public static final int kEncoderVelocitySignalPeriodMs = 271;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 8.0;

  /** Min voltage (negative = reverse). */
  public static final double kMinVoltage = -kMaxVoltage;

  /** Voltage in Idle state. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage in Intaking state (positive = pull in). */
  public static final double kIntakingVoltage = 5.0;

  /** Voltage magnitude in Reversing state (negative = spit out). */
  public static final double kReversingVoltage = -2.0;

  /** Delta volts per step. */
  public static final double kStepVolts = 0.25;
}
