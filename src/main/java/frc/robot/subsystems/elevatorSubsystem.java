package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.RobotConstants;

public class elevatorSubsystem extends SubsystemBase {
    public PIDController pid = new PIDController(RobotConstants.kPInOut, RobotConstants.kIInOut, RobotConstants.kDInOut);
    private SparkMax elevatorMotor = new SparkMax(RobotConstants.elevatorDeviceID, MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    public double setpoint = 0;

    public elevatorSubsystem() {}

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Set point", setpoint);
        SmartDashboard.putNumber("Encoder Reading",elevatorEncoder.getPosition());
        setMotor(0.0);
        toPID(setpoint);
    }

    private void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    private double getPosition(){
        //need to aassign this to a vlue that you return
        double distance = elevatorEncoder.getPosition() * 13 * 42 * 10;
        // motor.get position * number
        return distance;
       // return distance (in cm or m)
    }

    private void toPID(double setpoint) {
        double kFFValue = Math.cos(((90 - getPosition())*(Math.PI/180))) * 0.05;
        
        double motorValue = (pid.calculate(getPosition(), setpoint) + kFFValue);
        if (motorValue < -0.1) {
            motorValue = -0.1;
        }
        if (motorValue > 0.1) {
            motorValue = 0.1;
        }
        elevatorMotor.set(motorValue);
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

    public Command elevatorCommand(double position) {
        return this.runOnce(() -> setPosition(position));
    }

    public Command elevatorManual(boolean direction) {
        return this.run(() -> iterateSetPoint(direction));
    }
}