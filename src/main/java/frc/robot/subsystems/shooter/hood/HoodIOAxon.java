package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

/**
 * Hood IO: PWM {@link Servo} (Axon or compatible) + absolute analog for feedback. {@link HoodIO} uses radians
 * elevation from horizontal ({@link HoodConstants#kMinAngleRad}…{@link HoodConstants#kMaxAngleRad}); {@link Servo#setAngle}
 * uses degrees (WPILib hobby range 0–180). Preconditions: (1) wire {@link HoodConstants#kServoId} / {@link
 * HoodConstants#kEncoderId} to RIO PWM and analog; (2) calibrate {@link HoodConstants#kAxonEncoderVoltsAtMinAngle} /
 * {@link HoodConstants#kAxonEncoderVoltsAtMaxAngle} at both hard stops; (3) if horn angle ≠ elevation, set
 * {@link HoodConstants#kAxonServoAngleOffsetDeg} and/or {@link HoodConstants#kAxonServoInverted}. CAN-only Axon stacks
 * need a different IO class.
 */
public class HoodIOAxon implements HoodIO {

  private static final double kServoSetAngleMinDeg = 0.0;
  private static final double kServoSetAngleMaxDeg = 180.0;

  private final Servo axonServo;
  private final AnalogInput axonEncoder;

  /** Subtracted from linearized encoder angle so {@link #resetEncoder} can re-zero reporting. */
  private double encoderOffsetRad = 0.0;

  private double prevPositionRad = Double.NaN;
  private double prevTimeSec = Double.NaN;

  public HoodIOAxon() {
    axonServo = new Servo(kServoId);
    axonEncoder = new AnalogInput(kEncoderId);
    // Hardware averages 2^2 samples per getAverageVoltage() read (reduces analog noise).
    axonEncoder.setAverageBits(2);
  } // End HoodIOAxon Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double volts = axonEncoder.getAverageVoltage();
    double rawElevationRad = encoderVoltsToElevationRad(volts);
    double positionRad = rawElevationRad - encoderOffsetRad;
    positionRad = MathUtil.clamp(positionRad, kMinAngleRad, kMaxAngleRad);

    double nowSec = Timer.getFPGATimestamp();
    if (!Double.isNaN(prevTimeSec)) {
      double dtSec = nowSec - prevTimeSec;
      inputs.velocityRadsPerSec = (dtSec > 1e-6) ? (positionRad - prevPositionRad) / dtSec : 0.0;
    } else {
      inputs.velocityRadsPerSec = 0.0;
    }
    prevPositionRad = positionRad;
    prevTimeSec = nowSec;

    inputs.motorConnected =
        volts >= -0.05
            && volts <= kAxonAnalogReferenceVolts + 0.5
            && Math.abs(kAxonEncoderVoltsAtMaxAngle - kAxonEncoderVoltsAtMinAngle) > 1e-3;
    inputs.positionRads = positionRad;
    // PWM servo: no controller-reported output volts or current; keep zero so logs stay numeric.
    inputs.appliedVolts = 0.0;
    inputs.supplyCurrentAmps = 0.0;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    double elRad = MathUtil.clamp(targetRads, kMinAngleRad, kMaxAngleRad);
    double deg = Units.radiansToDegrees(elRad);
    if (kAxonServoInverted) {
      deg =
          Units.radiansToDegrees(kMinAngleRad) + Units.radiansToDegrees(kMaxAngleRad) - deg;
    }
    deg += kAxonServoAngleOffsetDeg;
    deg = MathUtil.clamp(deg, kServoSetAngleMinDeg, kServoSetAngleMaxDeg);
    axonServo.setAngle(deg);
  } // End setTargetPosition

  @Override
  public void resetEncoder() {
    double volts = axonEncoder.getAverageVoltage();
    encoderOffsetRad = encoderVoltsToElevationRad(volts) - kDisabledAngleRad;
    prevPositionRad = Double.NaN;
    prevTimeSec = Double.NaN;
  } // End resetEncoder

  @Override
  public void stop() {
    setTargetPosition(kDisabledAngleRad);
  } // End stop

  /** Maps absolute analog voltage to hood elevation (rad) using two-point calibration in {@link HoodConstants}. */
  private static double encoderVoltsToElevationRad(double volts) {
    double minV = kAxonEncoderVoltsAtMinAngle;
    double maxV = kAxonEncoderVoltsAtMaxAngle;
    double span = maxV - minV;
    if (Math.abs(span) < 1e-6) {
      return kMinAngleRad;
    }
    // 0 = at kAxonEncoderVoltsAtMinAngle, 1 = at kAxonEncoderVoltsAtMaxAngle (span may be negative).
    double voltageFractionFromMinToMax = (volts - minV) / span;
    voltageFractionFromMinToMax = MathUtil.clamp(voltageFractionFromMinToMax, 0.0, 1.0);
    return kMinAngleRad + voltageFractionFromMinToMax * (kMaxAngleRad - kMinAngleRad);
  } // End encoderVoltsToElevationRad
}
