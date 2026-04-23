package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the Turret (position-controlled targeting) subsystem. */
public final class TurretConstants {

  private TurretConstants() {}

  /** CAN ID of the Turret motor (NEO 550 on SPARK MAX or Kraken on Talon FX). */
  public static final int kMotorId = 54;

  /** Idle behavior when output is zero (coast or brake). SPARK MAX only. */
  public static final SparkBaseConfig.IdleMode kIdleMode = SparkBaseConfig.IdleMode.kCoast;

  /** Neutral mode when output is zero (coast or brake). Talon FX only. */
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

  /** Smart current limit. SPARK MAX only. */
  public static final int kSmartCurrentLimitAmps = 25;

  /** Stator current limit. Talon FX only. */
  public static final double kStatorCurrentLimitAmps = 30.0;

  /** Turret radians per motor rotation (output / input). 1.0 = 1:1. */
  public static final double kGearRatio = 84.545454;

  /** PID gains for onboard position control. */
  public static final double kP = 14.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  /** Period for sending signals to the motor. SPARK MAX only. */
  public static final int kSignalsPeriodMs = 19;
  public static final int kEncoderVelocitySignalPeriodMs = 19;
  
  /** Default aim direction in robot frame (0 = forward); π = toward robot back. */
  public static final Rotation2d kDefaultAimDirectionRobotFrame = Rotation2d.kPi;

  /** Minimum Turret angle. */
  public static final double kMinAngleRad = Units.degreesToRadians(-175.0);

  /** Maximum Turret angle. */
  public static final double kMaxAngleRad = Units.degreesToRadians(190.0);

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance for considering the Turret on target (setpoint vs measured, Turret frame). */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(2.0);

  /** 
   * Tolerance for considering the Turret on target (setpoint vs measured, Turret frame).
   * Larger tolerance than kAtTargetToleranceRad
   */
  public static final double kAtTargetToleranceNonHubRad = Units.degreesToRadians(8.0);

  /** Delta Rad per step. */
  public static final double kStepRad = Units.degreesToRadians(5.0);

  /**
   * Sim-only cap on commanded turret angular rate (rad/s). Toy plant in {@link frc.robot.subsystems.shooter.turret.TurretIOSim};
   * set high enough that lag is not dominated by this limit when testing spin tracking.
   */
  public static final double kSimMaxTurretRadPerSec = 12.0;
}
