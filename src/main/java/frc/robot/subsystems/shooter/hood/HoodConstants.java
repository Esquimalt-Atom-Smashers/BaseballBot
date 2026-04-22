package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.util.Units;

/** Constants for the Hood (position-controlled Shooter angle) subsystem. */
public final class HoodConstants { // XXX: Add correct values

  private HoodConstants() {}

  /** CAN ID of the Hood motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 55;

  /** PWM ID of the Hood servo. */
  public static final int kServoId = 0;

  /** Analog input ID of the Hood encoder. */
  public static final int kEncoderId = 1;

  /** Axon servo value (0–1) when hood is at 0° (shot straight out forward of the robot, parallel to the ground). */
  public static final double kServoSetAt0deg = 0.42;

  /** Axon servo value (0–1) when hood is at 90° (shot straight up along field +Z). */
  public static final double kServoSetAt90deg = 0.58;

  /**
   * Precomputed slope for {@link HoodIOAxon}: turns a hood elevation setpoint (rad, 0 = out … π/2 = up)
   * into a {@code Servo.set} value (0–1) using the two calibrated endpoints {@link #kServoSetAt0deg} and {@link #kServoSetAt90deg}.
   */
  public static final double kServoSetPerHoodAngleRad = (kServoSetAt90deg - kServoSetAt0deg) / (Math.PI / 2.0);

  /** Axon analog voltage when hood is at 0° (shot forward, parallel to the ground). */
  public static final double kAnalogVoltsAt0deg = 2.5;

  /** Axon analog voltage when hood is at 90° (shot straight up, +Z). */
  public static final double kAnalogVoltsAt90deg = 1.4;

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

  /** Hood elevation when the Hood is disabled/locked. */
  public static final double kDisabledAngleRad = Units.degreesToRadians(80.0);

  /** Minimum travel elevation (deg from horizontal); shallower shot. */
  public static final double kMinAngleRad = Units.degreesToRadians(50.0);

  /** Maximum travel elevation (deg from horizontal); steeper shot. */
  public static final double kMaxAngleRad = Units.degreesToRadians(80.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance for considering the Hood at target (measured vs target). */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);

  /** Sim only: max Hood setpoint slew rate. */
  public static final double kSimMaxSlewRadPerSec = Units.degreesToRadians(60.0);

  /** Delta Rad per step. */
  public static final double kStepAngleRads = Units.degreesToRadians(2.0);
}
