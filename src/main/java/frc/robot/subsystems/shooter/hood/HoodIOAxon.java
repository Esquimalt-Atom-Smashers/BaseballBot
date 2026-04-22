package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;

/** Hood IO using an Axon Servo and Analog Input. */
public class HoodIOAxon implements HoodIO {
  private final Servo axonServo;
  private final AnalogInput axonEncoder;

  /** Span of hood angle from 0° to 90° in radians. */
  private static final double kHoodAngle90DegInRad = Math.PI / 2.0;

  public HoodIOAxon() {
    axonServo = new Servo(kServoId);
    axonEncoder = new AnalogInput(kEncoderId);
  } // End HoodIOAxon Constructor

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double hoodAnalogVolts = axonEncoder.getAverageVoltage();
    double analogVoltsSpan90Minus0 = kAnalogVoltsAt90deg - kAnalogVoltsAt0deg;
    if (Math.abs(analogVoltsSpan90Minus0) < 1e-4) {
      inputs.positionRads = 0.0;
    } else {
      double hoodAngleRad =
          (hoodAnalogVolts - kAnalogVoltsAt0deg)
              / analogVoltsSpan90Minus0
              * kHoodAngle90DegInRad;
      inputs.positionRads = MathUtil.clamp(hoodAngleRad, 0.0, kHoodAngle90DegInRad);
    }
    inputs.analogVolts = hoodAnalogVolts;
  } // End updateInputs

  @Override
  public void setTargetPosition(double targetRads) {
    axonServo.set(servoSetForHoodAngleRad(targetRads));
  } // End setTargetPosition

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
   * Maps hood angle (rad) to {@code Servo.set} [0, 1]: 0 rad = 0° (forward, parallel to ground), π/2
   * rad = 90° (+Z), linear between {@link HoodConstants#kServoSetAt0deg} and {@link
   * HoodConstants#kServoSetAt90deg}.
   */
  private static double servoSetForHoodAngleRad(double hoodAngleRad) {
    double servoSetpointUnclamped = kServoSetAt0deg + kServoSetPerHoodAngleRad * hoodAngleRad;
    return MathUtil.clamp(servoSetpointUnclamped, 0.0, 1.0);
  } // End servoSetForHoodAngleRad
}
