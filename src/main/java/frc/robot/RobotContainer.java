package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.algaeSubsystem;
import frc.robot.subsystems.coralSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.inOutSubsystem;

// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
        TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
    );

    public static final algaeSubsystem algae = new algaeSubsystem();
    public static final coralSubsystem coral = new coralSubsystem();
    public static final elevatorSubsystem elevator  = new elevatorSubsystem();
    public static final inOutSubsystem inOut  = new inOutSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Drivetrain
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(1) * 0.2) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(0) * 0.2) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.button(3).whileTrue(drivetrain.applyRequest(() -> brake)); //Y
        joystick.button(2).whileTrue(drivetrain.applyRequest(() -> // B
            point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0)))
        ));

        // joystick.x().onTrue(
        //     parallel(
        //             elevatorSubsystem.setMotor(TunerConstants.elevatorSpeed),
        //             elevatorCommand.runCommand())
        //         .withName("Elevator"));
        // joystick.y().onTrue(
        //     parallel(
        //             coralSubsystem.setMotor(TunerConstants.coralSpeed),
        //             coralCommand.runCommand())
        //         .withName("Coral"));
        
        // algae.proximitySensorTrigger.whileTrue(algae.algaeCommand(-0.2));

        joystick.button(6).whileTrue(algae.algaeCommand(0.4));
        joystick.button(5).whileTrue(algae.algaeCommand(-0.4));

        joystick.button(7).whileTrue(coral.coralCommand(-0.4));
        joystick.button(8).whileTrue(coral.coralCommand(-0.4));

        joystick.povUp().onTrue(inOut.inOutCommand(-45));
        joystick.povLeft().onTrue(inOut.inOutCommand(0));

        joystick.button(1).whileTrue(inOut.inOutManual(true));
        joystick.button(4).whileTrue(inOut.inOutManual(false));

        joystick.button(5).whileTrue(coral.coralCommand(0.2));
        joystick.button(6).whileTrue(coral.coralCommand(-0.2));


        // joystick.button(9).whileTrue(elevator.elevatorCommand(30));
        // joystick.button(10).whileTrue(elevator.elevatorCommand(-30));


        // reset the field-centric heading on START
        joystick.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
