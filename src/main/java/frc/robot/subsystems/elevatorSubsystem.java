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
    public PIDController pid = new PIDController(RobotConstants.kPInOutE, RobotConstants.kIInOutE, RobotConstants.kDInOutE);
    private SparkMax elevatorMotor = new SparkMax(RobotConstants.elevatorDeviceID, MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    public double setpoint = 0;
    public double motorValueHere;
    public double pidkFFValue;

    public elevatorSubsystem() {}

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Elevator Set point", setpoint);
        SmartDashboard.putNumber("Elevator Encoder Reading", getPosition());
        SmartDashboard.putNumber("Elevator PID Motor Value", motorValueHere);
        SmartDashboard.putNumber("Elevator Pid kFF Value", pidkFFValue);
        
        setMotor(0.0);
        toPID(setpoint);
    }

    private void setMotor(double speed) {
        elevatorMotor.set(speed);
    }

    private double getPosition(){
        //need to aassign this to a vlue that you return
        //double distance = elevatorEncoder.getPosition() * 13 * 42 * 10;
        double distance = -elevatorEncoder.getPosition(); //negative?
        return distance;   // return distance (in cm or m)
    }

    private void toPID(double setpoint) {
        double kFFValue = Math.cos(((90 - getPosition())*(Math.PI/180))) * 0.05;
        
        pidkFFValue = kFFValue;

        double motorValue = ((pid.calculate(getPosition(), setpoint) + kFFValue)); //temp please what

        motorValueHere = motorValue;

        if (motorValue < -0.2) {
            motorValue = -0.2;
        }
        if (motorValue > 0.2) {
            motorValue = 0.;
        }
        elevatorMotor.set(motorValue);
    }
    
    private void iterateSetPoint(boolean direction) {
        if (direction){
            setpoint++;
            //elevatorMotor.set(-0.4);
        }
        else{
            setpoint--;
            //elevatorMotor.set(+0.4);
            // this was up
        }
    }

    // private void setSetpoint(double position) {
    //     setpoint = position;
    // }

    // public Command elevatorCommand(double position) {
    //     return this.runOnce(() -> setSetpoint(position));
    // }

    public Command elevatorManual(boolean direction) {
        return this.run(() -> iterateSetPoint(direction));
    }
}