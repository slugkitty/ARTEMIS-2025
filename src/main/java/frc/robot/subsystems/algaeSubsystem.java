package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class algaeSubsystem extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private SparkMax algaeMotorLeft = new SparkMax(RobotConstants.algaeDeviceID1, MotorType.kBrushed);
    private SparkMax algaeMotorRight = new SparkMax(RobotConstants.algaeDeviceID2, MotorType.kBrushed);    

   // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    
    //TRigger for when the algae is in the intake
    public Trigger proximitySensorTrigger;

    // Sets default to motors set to 0
    // Creates trigger based on algaeBoolean function
    public algaeSubsystem() {
        setDefaultCommand(run(() -> {
            setMotor(0.0);
        }));
        //proximitySensorTrigger = new Trigger(this::algaeBoolean);
    }

    @Override
    public void periodic() {
        setMotor(0.0);
    }

    public void setMotor(double speed) {
        algaeMotorLeft.set(-speed);
        algaeMotorRight.set(speed); //negative
    }

    // Determines algae distance from sensor
    // public boolean algaeBoolean() {
    //     if (m_colorSensor.getProximity() >= 500){
    //         return true;
    //     }
    //     else {
    //         return false;
    //     }
    // }

    public Command algaeCommand(double speed) {
        return this.run(() -> setMotor(speed));
    }
}