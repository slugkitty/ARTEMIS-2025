package frc.robot.subsystems;

import frc.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj2.command.Command;

public class climberSubsystem extends SubsystemBase {

    private TalonFXS climbermotor = new TalonFXS(RobotConstants.climberDeviceID);

    public climberSubsystem() {
        setDefaultCommand(run(() -> {
            setMotor(0.0);
        }));
    }

    public void setMotor(double speed) {
        climbermotor.set(speed);
    }

    @Override
    public void periodic() {
    }

    public Command climberCommand(double speed) {
        return this.run(() -> setMotor(speed));
    }
}
