package frc.robot.subsystems.shooter.transfer;

import com.revrobotics.spark.config.SparkBaseConfig;

/** Constants for the Transfer (agitator-to-shooter) subsystem. */
public final class TransferConstants {

  private TransferConstants() {}

  /** CAN ID of the Transfer motor. */
  public static final int kMotorId = 53;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Set true if positive voltage spins the Transfer the opposite direction. */
  public static final boolean kMotorInverted = true;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;
  
  /** Open-loop ramp time from 0 to full output. Limits current spikes on step changes. SPARK MAX only. */
  public static final double kOpenLoopRampRateSec = 0.0;

  /** Period for sending signals to the motor. */
  public static final int kSignalsPeriodMs = 107;
  public static final int kEncoderVelocitySignalPeriodMs = 251;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 10.0;

  /** Voltage in Idle state. */
  public static final double kIdleVoltage = 0.0;

  /** Voltage in Staging state (slow pre-load). */
  public static final double kStagingVoltage = 2.0;

  /** Voltage in Shooting state. */
  public static final double kShootingVoltage = 8.0;

  /** Proximity at or above this value = ball present (REV Color Sensor V3: 0–2047). */
  public static final int kColorSensorProximityThreshold = 150;

  /** Delta volts per step. */
  public static final double kStepVolts = 0.25;
}
