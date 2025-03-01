package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.generated.TunerConstants;
import frc.robot.constants.RobotConstants;

public class coralSubsystem extends SubsystemBase {
    private SparkMax coralMotor = new SparkMax(RobotConstants.coralDeviceID, MotorType.kBrushed);
    //private final boolean open;

    public coralSubsystem() {
        setDefaultCommand(run(() -> {
            setMotor(0.0);
        }));
     }

     @Override
     public void periodic() {
     }
 
     public void setMotor(double speed) {
         coralMotor.set(speed);
     }

     public Command coralCommand(double speed) {
         return this.run(() -> setMotor(speed));
     }
 }