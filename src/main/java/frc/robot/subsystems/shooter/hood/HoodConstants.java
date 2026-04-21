package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/** Constants for the Hood (position-controlled Shooter angle) subsystem. */
public final class HoodConstants { // XXX: Add correct values

  private HoodConstants() {}

  /** CAN ID of the Hood motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 55;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kBrake;

  /** Set true if positive output moves the Hood the opposite direction. */
  public static final boolean kMotorInverted = false;

  /** Neutral mode when output is zero (coast or brake). Talon FX only. */
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 3;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;

  /** Hood radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 1.0;

  /** PID gains for onboard position control and for sim software control. */
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Period for sending signals to the motor. SPARK MAX only. */
  public static final int kSignalsPeriodMs = 19;
  public static final int kEncoderVelocitySignalPeriodMs = 19;

  /** Hood angle when the Hood is disabled/locked. */
  public static final double kDisabledAngleRad = Units.degreesToRadians(25.0);

  /** Minimum Hood angle. */
  public static final double kMinAngleRad = Units.degreesToRadians(25.0);
  
  /** Maximum Hood angle. */
  public static final double kMaxAngleRad = Units.degreesToRadians(10.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance for considering the Hood at target (measured vs target). */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);

  /** Sim only: max Hood setpoint slew rate. */
  public static final double kSimMaxSlewRadPerSec = Units.degreesToRadians(60.0);
}
