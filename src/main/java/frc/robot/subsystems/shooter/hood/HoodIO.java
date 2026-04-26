package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Hood (one motor, position controlled). */
public interface HoodIO {

  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double analogVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(HoodIOInputs inputs) {}

  /** Set the target position. */
  default void setTargetPosition(double targetRads) {}

  /** Set the target set position */
  default void setServoPosition(double targetSetPosition) {}

  /** Sets the encoder position to 0. */
  default void resetEncoder() {}

  /** Stop the motor (brake). */
  default void stop() {}
}
