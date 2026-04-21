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

  /**
   * RoboRIO analog reference (V) used only for sanity checks on {@link HoodIOAxon}; position uses
   * {@link #kAxonEncoderVoltsAtMinAngle} / {@link #kAxonEncoderVoltsAtMaxAngle}.
   */
  public static final double kAxonAnalogReferenceVolts = 5.0;

  /** Analog voltage read when the hood is at {@link #kMinAngleRad}. */
  public static final double kAxonEncoderVoltsAtMinAngle = 0.0;

  /** Analog voltage read when the hood is at {@link #kMaxAngleRad}. */
  public static final double kAxonEncoderVoltsAtMaxAngle = 5.0;

  /**
   * When true, {@link edu.wpi.first.wpilibj.Servo#setAngle} is reflected about mid-travel so increasing commanded
   * hood elevation decreases PWM angle (use if horn or cable runs opposite).
   */
  public static final boolean kAxonServoInverted = false;

  /**
   * Added to the servo angle (deg) after invert. Default assumes servo command degrees match hood elevation
   * degrees; tune if the horn is not 1:1 with launch elevation.
   */
  public static final double kAxonServoAngleOffsetDeg = 0.0;

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
