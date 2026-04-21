package frc.robot.subsystems.agitator;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the Agitator (intake-to-transfer) subsystem. */
public final class AgitatorConstants {

  private AgitatorConstants() {}

  /** CAN ID of the Agitator motor. */
  public static final int kMotorId = 52;

  /** Idle behavior when output is zero (coast or brake). */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Set true if positive voltage spins the Agitator the opposite direction. */
  public static final boolean kMotorInverted = true;

  /** Smart current limit. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Open-loop ramp time from 0 to full output. Limits current spikes on step changes. */
  public static final double kOpenLoopRampRateSec = 0.0;

  /** Period for sending signals to the motor. */
  public static final int kSignalsPeriodMs = 107;
  public static final int kEncoderVelocitySignalPeriodMs = 251;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 11.0;

  /** Voltage in Idle state. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage in Staging state (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage in Shooting state (fast loading). */
  public static final double kShootingVoltage = 9.5;

  /** Delta volts per step. */
  public static final double kStepVolts = 0.25;
}
