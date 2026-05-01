package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Turret (one motor, position controlled). */
public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Update inputs from the hardware. */
  default void updateInputs(TurretIOInputs inputs) {}

  /** Set the target position. */
  default void setTargetPosition(double targetRads) {}

  /**
   * Set target position and velocity feedforward (turret rad/s for spin compensation).
   * Default ignores velocity; Talon uses native vel FF; Spark uses arbitrary FF volts from rate.
   */
  default void setTargetPosition(double targetRads, double velocityFeedforwardRadPerSec) {
    setTargetPosition(targetRads);
  }

  /**
   * Set target position with velocity feedforward and precomputed arbitrary feedforward volts.
   * Default ignores velocity and FF volts.
   */
  default void setTargetPosition(
      double targetRads, double velocityFeedforwardRadPerSec, double velocityFFVolts) {
    setTargetPosition(targetRads, velocityFeedforwardRadPerSec);
  }

  /** Sets the encoder position to 0. */
  default void resetEncoder() {}

  /** Stop the motor. */
  default void stop() {}
}
