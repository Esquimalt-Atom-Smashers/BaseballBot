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
    double hoodAnalogVolts = axonEncoder.getAverageVoltage();
    double analogVoltsSpanHighMinus0 = kAnalogVoltsAt80deg - kAnalogVoltsAt0deg;
    if (Math.abs(analogVoltsSpanHighMinus0) < 1e-4) {
      inputs.positionRads = 0.0;
    } else {
      double hoodAngleRad =
          (hoodAnalogVolts - kAnalogVoltsAt0deg)
              / analogVoltsSpanHighMinus0
              * kAnalogServoCalibHighAngleRad;
      inputs.positionRads = MathUtil.clamp(hoodAngleRad, 0.0, kAnalogServoCalibHighAngleRad);
    }
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
}
