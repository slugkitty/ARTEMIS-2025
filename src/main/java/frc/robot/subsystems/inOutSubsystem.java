package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.RobotConstants;

public class inOutSubsystem extends SubsystemBase {
    public PIDController pid = new PIDController(RobotConstants.kPInOut, RobotConstants.kIInOut, RobotConstants.kDInOut);
    private SparkMax inOutMotor = new SparkMax(RobotConstants.InOutMotorDeviceID, MotorType.kBrushless);
    private RelativeEncoder inOutEncoder = inOutMotor.getEncoder();
    public double setpoint = 0;

    public inOutSubsystem() {}

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Set point", setpoint);
        SmartDashboard.putNumber("Current Angle", motorAngle());
        SmartDashboard.putNumber("Encoder Reading",inOutEncoder.getPosition());
        setMotor(0.0);
        toPID(setpoint);
    }

    // Gives encoder in degrees (hopefully)
    private double motorAngle() {
        return inOutEncoder.getPosition() * 10;
    }

    private void toPID(double setpoint) {
        double kFFValue = Math.cos(((90 - motorAngle())*(Math.PI/180))) * 0.05;
        
        double motorValue = (pid.calculate(motorAngle(), setpoint) + kFFValue);
        if (motorValue < -0.1) {
            motorValue = -0.1;
        }
        if (motorValue > 0.1) {
            motorValue = 0.1;
        }
        inOutMotor.set(motorValue);
    }
    
    private void setMotor(double speed) {
        inOutMotor.set(speed);
    }

    private void iterateSetPoint(boolean direction) {
        if (direction){
            setpoint++;
        }
        else{
            setpoint--;
        }
    }

    private void setPosition(double position) {
        setpoint = position;
    }

    public Command inOutCommand(double position) {
        return this.runOnce(() -> setPosition(position));
    }

    public Command inOutManual(boolean direction) {
        return this.run(() -> iterateSetPoint(direction));
    }
}