package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

/** Hood IO using an Axon Servo and Analog Input. */
public class HoodIOAxon implements HoodIO {
  private final Servo axonServo;
  private final AnalogInput axonEncoder;

  public HoodIOAxon() {
    axonServo = new Servo(kServoId);
    axonEncoder = new AnalogInput(kEncoderId);
  } // End HoodIOAxon Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // PWM hobby servo does not provide onboard connection telemetry.
    inputs.motorConnected = false;
    double hoodAnalogVolts = axonEncoder.getAverageVoltage();
    inputs.positionRads = analogVoltsToHoodAngleRad(hoodAnalogVolts);
    inputs.analogVolts = hoodAnalogVolts;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    axonServo.set(servoSetForHoodAngleRad(targetRads));
  } // End setTargetPosition

  @Override
  public void setServoPosition(double targetSetPosition) {
    axonServo.set(targetSetPosition);
  }

  /**
   * Stops driving the PWM pin ({@link edu.wpi.first.wpilibj.PWM#setDisabled}), so the Rio is not
   * commanding a pulse width. The next {@link Servo#set(double)} re-enables the channel. This is
   * not the same as removing servo supply power; for that use wiring (e.g. relay on the servo bus).
   */
  @Override
  public void stop() {
    axonServo.setDisabled();
  } // End stop

  /**
   * Maps hood angle (rad from horizontal) to {@code Servo.set} [0, 1]: linear from {@link HoodConstants#kServoSetAt0deg}
   * at 0 rad to {@link HoodConstants#kServoSetAt80deg} at {@link HoodConstants#kAnalogServoCalibHighAngleRad}.
   */
  private static double servoSetForHoodAngleRad(double hoodAngleRad) {
    double servoSetpointUnclamped = kServoSetAt0deg + kServoSetPerHoodAngleRad * hoodAngleRad;
    return MathUtil.clamp(servoSetpointUnclamped, 0.0, 1.0);
  } // End servoSetForHoodAngleRad

  /** Maps measured analog voltage to hood angle (rad) using piecewise linear interpolation. */
  private static double analogVoltsToHoodAngleRad(double volts) {
    int pointCount = Math.min(kAnalogAngleBreakpointsRad.length, kAnalogVoltsBreakpoints.length);
    if (pointCount < 2) {
      return 0.0;
    }

    double firstVolts = kAnalogVoltsBreakpoints[0];
    double lastVolts = kAnalogVoltsBreakpoints[pointCount - 1];
    if (firstVolts > lastVolts) {
      if (volts >= firstVolts) {
        return kAnalogAngleBreakpointsRad[0];
      }
      if (volts <= lastVolts) {
        return kAnalogAngleBreakpointsRad[pointCount - 1];
      }
    } else {
      if (volts <= firstVolts) {
        return kAnalogAngleBreakpointsRad[0];
      }
      if (volts >= lastVolts) {
        return kAnalogAngleBreakpointsRad[pointCount - 1];
      }
    }

    for (int i = 0; i < pointCount - 1; i++) {
      double v0 = kAnalogVoltsBreakpoints[i];
      double v1 = kAnalogVoltsBreakpoints[i + 1];
      boolean inSegment = (v0 >= v1) ? (volts <= v0 && volts >= v1) : (volts >= v0 && volts <= v1);
      if (!inSegment) {
        continue;
      }

      double a0 = kAnalogAngleBreakpointsRad[i];
      double a1 = kAnalogAngleBreakpointsRad[i + 1];
      double dv = v1 - v0;
      if (Math.abs(dv) < 1e-6) {
        return a0;
      }
      double t = (volts - v0) / dv;
      return a0 + t * (a1 - a0);
    }

    return kAnalogAngleBreakpointsRad[pointCount - 1];
  } // End analogVoltsToHoodAngleRad
}
