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
  public static final double kServoSetAt0deg = 0.7613333333333335;

  /** Axon servo value (0–1) when hood is at 80°. */
  public static final double kServoSetAt80deg = 0.5195555555555554;

  /**
   * Upper hood elevation (rad from horizontal) used for analog + servo calibration. Low endpoint is 0 rad; high
   * endpoint readings match {@link #kServoSetAt80deg} and {@link #kAnalogVoltsAt80deg}.
   */
  public static final double kAnalogServoCalibHighAngleRad = Units.degreesToRadians(80.0);

  /**
   * Precomputed slope for {@link HoodIOAxon}: hood elevation (rad, 0 = forward … {@link #kAnalogServoCalibHighAngleRad} = calibrated high)
   * to {@code Servo.set} (0–1) using {@link #kServoSetAt0deg} and {@link #kServoSetAt80deg}.
   */
  public static final double kServoSetPerHoodAngleRad = (kServoSetAt80deg - kServoSetAt0deg) / kAnalogServoCalibHighAngleRad;

  /** Axon analog voltage when hood is at 0° (shot forward, parallel to the ground). */
  public static final double kAnalogVoltsAt0deg = 2.4;

  /** Axon analog voltage when hood is at {@link #kAnalogServoCalibHighAngleRad} (same mechanical pose as {@link #kServoSetAt80deg}). */
  public static final double kAnalogVoltsAt80deg = 1.715;

  // TODO: Add correct values
  /** Axon analog voltage when hood is at 75°. */
  public static final double kAnalogVoltsAt75deg = 1.7578125;

  /** Axon analog voltage when hood is at 70°. */
  public static final double kAnalogVoltsAt70deg = 1.800625;

  /** Axon analog voltage when hood is at 65°. */
  public static final double kAnalogVoltsAt65deg = 1.8434375;

  /** Axon analog voltage when hood is at 60°. */
  public static final double kAnalogVoltsAt60deg = 1.88625;

  /** Axon analog voltage when hood is at 55°. */
  public static final double kAnalogVoltsAt55deg = 1.9290625;

  /** Axon analog voltage when hood is at 50°. */
  public static final double kAnalogVoltsAt50deg = 1.971875;

  /** Ordered hood-angle breakpoints for Axon analog interpolation (rad). */
  public static final double[] kAnalogAngleBreakpointsRad = {
    Units.degreesToRadians(0.0),
    Units.degreesToRadians(50.0),
    Units.degreesToRadians(55.0),
    Units.degreesToRadians(60.0),
    Units.degreesToRadians(65.0),
    Units.degreesToRadians(70.0),
    Units.degreesToRadians(75.0),
    Units.degreesToRadians(80.0)
  };

  /** Ordered Axon analog voltage breakpoints corresponding to {@link #kAnalogAngleBreakpointsRad}. */
  public static final double[] kAnalogVoltsBreakpoints = {
    kAnalogVoltsAt0deg,
    kAnalogVoltsAt50deg,
    kAnalogVoltsAt55deg,
    kAnalogVoltsAt60deg,
    kAnalogVoltsAt65deg,
    kAnalogVoltsAt70deg,
    kAnalogVoltsAt75deg,
    kAnalogVoltsAt80deg
  };

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

  /** Maximum travel elevation (rad from horizontal); matches analog/servo high calibration. */
  public static final double kMaxAngleRad = kAnalogServoCalibHighAngleRad;

  /** Max voltage magnitude applied to the motor. */
  public static final double kMaxVoltage = 12.0;

  /** Tolerance for considering the Hood at target (measured vs target). */
  public static final double kAtTargetToleranceRad = Units.degreesToRadians(1.0);

  /** Sim only: max Hood setpoint slew rate. */
  public static final double kSimMaxSlewRadPerSec = Units.degreesToRadians(60.0);

  /** Delta Rad per step. */
  public static final double kStepAngleRads = Units.degreesToRadians(2.0);
}
