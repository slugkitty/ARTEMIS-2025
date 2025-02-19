package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.TunerConstants;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class elevatorSubsystem extends SubsystemBase {
    // public double kP = 0.3;
    // public double kI = 0.0;
    // public double kD = 0.005;
    // public PIDController pid = new PIDController(kP, kI, kD);
    // Encoder encoderLift = new Encoder(4,5);
    // encoderLift.setMinRate(2);
    // encoderLift.setReverseDirection(false);
    // encoderLift.setDistancePerPulse(Constants.Pivot_Encoder_Scale_Factor);
    // encoderLift.setSamplesToAverage(5);
    // private SparkMax elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    // Encoder encoderLift = new Encoder(4,5);
    // public static double setpoint = 0;


    // public elevatorSubsystem() {
    // }

    // // what is the motor value
    // // if motor value isn't hte speed of hte motor, how do i set the setpoint along side the speed of hte motor
    // //also how do i import a NEO motor

    // @Override
    // public void periodic() {
    //     PIDArm();
    //     toPID(setpoint);
    // }
    //     public static void incrementAngle(double increment) {
    //         setpoint += increment;
    //     }
    //     public void toPID(double setpoint){
    //         double kFFValue = Math.cos(((90 - encoderLift.getDistance())*(Math.PI/180))) * 0.05;
            
    //         double motorValue = pid.calculate(encoderLift.getDistance(), setpoint) + kFFValue;
    //         if (motorValue < -0.5) {
    //           motorValue = -0.5;
    //         }
    //         if (motorValue > 0.5) {
    //           motorValue = 0.5;
    //         }
    //         elevatorMotor.set(motorValue);
    //     }
     

    // public Command elevatorCommand(double setpoint) {
    //     return this.runOnce(() -> setMotor(setpoint));
    // }
}
