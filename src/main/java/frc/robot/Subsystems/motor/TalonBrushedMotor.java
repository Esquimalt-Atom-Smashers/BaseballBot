package frc.robot.Subsystems.motor;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class TalonBrushedMotor extends SubsystemBase {
    private TalonSRX talonMotor;
    private Timer printTimer = new Timer();

    // Initialize REV through Bore Encoder on DIO Port 0
    //Using DutyCycleEncoder for Absolute Mode
    private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);

    public TalonBrushedMotor(int canID, boolean isInverted) {
        // set the motor to factory default to start from a known state
        talonMotor = new TalonSRX(canID);
        talonMotor.configFactoryDefault();

        // can reverse motor direction if needed
        talonMotor.setInverted(isInverted);
        printTimer.start();
    }

    public void setPower(double power) {
        talonMotor.set(ControlMode.PercentOutput, power);
    }

    public Command setPowerCommand(double power) {
        // requires it to extend subsystem
        return runOnce(() -> {talonMotor.set(ControlMode.PercentOutput, power);});
    }
    @Override
    public void periodic() {
        // this will be called once per scheduled run

        if (printTimer.hasElapsed(1.0)) {
            double position = absEncoder.get();
            System.out.println("Encoder Absolute Position: " + position);
            printTimer.reset();
        }
    }
    
    @Override
    public void simulationPeriodic() {
        //called once per run during simulation
    }
}
