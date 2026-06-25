package frc.robot.Subsystems;

import frc.robot.Constants;
import com.revrobotics.REVLibError;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    public Drive(SparkMax leftMotor, SparkMax rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void arcadeDrive(double forward, double turn) {
        leftMotor.set(forward + turn);
        rightMotor.set(forward - turn);
    }
}
