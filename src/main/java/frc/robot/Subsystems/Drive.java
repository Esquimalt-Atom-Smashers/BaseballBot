package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Subsystems.motor.TalonBrushedMotor;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final TalonBrushedMotor leftMotor;

    public Drive(TalonBrushedMotor leftMotor) {
        this.leftMotor = leftMotor;
        // this.rightMotor = rightMotor;
    }

    public void arcadeDrive(double forward, double turn) {
        leftMotor.setPower(forward);
        // rightMotor.set(forward - turn);
    }
}
