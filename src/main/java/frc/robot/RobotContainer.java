package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.algaeSubsystem;
import frc.robot.subsystems.climberSubsystem;
import frc.robot.subsystems.coralSubsystem;
import frc.robot.subsystems.elevatorSubsystem;
import frc.robot.subsystems.inOutSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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

    private final CommandJoystick driverJoystick = new CommandJoystick(0);
    private final CommandJoystick operatorJoystick = new CommandJoystick(1);

    // private final CommandPS5Controller driverJoystick = new CommandPS5Controller(0);
    // private final CommandPS5Controller operatorJoystick = new CommandPS5Controller(0);

    public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
        TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
    );

    public static final algaeSubsystem algae = new algaeSubsystem();
    public static final coralSubsystem coral = new coralSubsystem();
    public static final elevatorSubsystem elevator  = new elevatorSubsystem();
    public static final inOutSubsystem inOut  = new inOutSubsystem();
    public static final climberSubsystem climber = new climberSubsystem();

    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // //autoChooser.addOption("Auto1", autoOne());
        // SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Drivetrain
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getRawAxis(1) * 0.2) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getRawAxis(0) * 0.2) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRawAxis(4) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        driverJoystick.button(3).whileTrue(drivetrain.applyRequest(() -> brake)); //Y
        driverJoystick.button(2).whileTrue(drivetrain.applyRequest(() -> // B
            point.withModuleDirection(new Rotation2d(-driverJoystick.getRawAxis(1), -driverJoystick.getRawAxis(0)))
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

        // TODO: Figure out how to make this work
        //joystick.axisMagnitudeGreaterThan(5, 0.2).whileTrue(algae.algaeCommand(0.4, -1));
        
        // ALGAE
        operatorJoystick.axisGreaterThan(1, 0.2).whileTrue(algae.algaeCommand(0.4));
        operatorJoystick.axisLessThan(1, -0.2).whileTrue(algae.algaeCommand(-0.4));

        // CORAL
        operatorJoystick.axisGreaterThan(5, 0.2).whileTrue(coral.coralCommand(0.4));
        operatorJoystick.axisLessThan(5, -0.2).whileTrue(coral.coralCommand(-0.4));

        // IN OUT
        operatorJoystick.button(4).whileTrue(inOut.inOutCommand(-45)); //Y
        operatorJoystick.button(3).whileTrue(inOut.inOutCommand(0)); //B

        driverJoystick.button(1).whileTrue(inOut.inOutManual(true));
        driverJoystick.button(2).whileTrue(inOut.inOutManual(false));

        // CLIMBER
        operatorJoystick.button(5).whileTrue(climber.climberCommand(0.4)); //Left Button
        operatorJoystick.button(6).whileTrue(climber.climberCommand(-0.4)); //Right Button

        // ELEVATOR
        // operatorJoystick.povUp().onTrue(elevator.elevatorCommand(45)); //POV Up
        // operatorJoystick.povDown().onTrue(elevator.elevatorCommand(0)); //POV Down

        driverJoystick.povUp().whileTrue(elevator.elevatorManual(true));
        driverJoystick.povDown().whileTrue(elevator.elevatorManual(false));

        // reset the field-centric heading on START
        driverJoystick.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverJoystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }


    // AUTOS

    // public Command AutoOne(){
    //     return Commands.sequence(
    //         driveTo(drivetrain.getPose(), 0,0,0)
    //     );
    // }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return Commands.print("No autonomous command configured");
        //return autoChooser.getSelected();
    }   
    
    // public Command driveTo(Pose2d currentPos, double x, double y, int degrees){      
    //     // The rotation component in these poses represents the direction of travel
    //     Pose2d startPos = new Pose2d(currentPos.getTranslation(), new Rotation2d());
    //     Pose2d endPos = new Pose2d(currentPos.getTranslation().plus(new Translation2d(x, y)), new Rotation2d(degrees* (Math.PI/180)));

    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
    //     PathPlannerPath path = new PathPlannerPath(
    //         waypoints, 
    //         new PathConstraints(
    //         4.0, 4.0, 
    //         Units.degreesToRadians(360), Units.degreesToRadians(540)
    //         ),
    //         null, // Ideal starting state can be null for on-the-fly paths
    //         new GoalEndState(0.0, endPos.getRotation())
    //     );
    //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //     path.preventFlipping = true;

    //     return AutoBuilder.followPath(path);
    // }
}
