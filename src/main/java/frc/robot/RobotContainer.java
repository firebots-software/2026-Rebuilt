// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.ClimbCommands.L1Climb;
import frc.robot.commandGroups.ClimbCommands.L2Climb;
import frc.robot.commandGroups.ClimbCommands.L3Climb;
import frc.robot.commandGroups.WarmUpAndShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private BooleanSupplier redside = () -> redAlliance;
  private static boolean redAlliance;

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController debugJoystick = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ClimberSubsystem climberSubsystem =
      Constants.climberOnRobot ? new ClimberSubsystem() : null;
  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem() : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron = Constants.shooterOnRobot ? new ShooterSubsystem() : null;

  private final AutoFactory autoFactory; // no marker

  public final AutoRoutine autoRoutine; // with markers

  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {

    // paths without marker
    autoFactory = drivetrain.createAutoFactory();

    Command redClimb =
        autoFactory
            .resetOdometry("RedClimb.traj")
            .andThen(autoFactory.trajectoryCmd("RedClimb.traj"));
    Command redDepot =
        autoFactory
            .resetOdometry("RedDepot.traj")
            .andThen(autoFactory.trajectoryCmd("RedDepot.traj"));
    Command redOutpost =
        autoFactory
            .resetOdometry("RedOutpost.traj")
            .andThen(autoFactory.trajectoryCmd("RedOutpost.traj"));
    Command moveForward =
        autoFactory
            .resetOdometry("MoveForward.traj")
            .andThen(autoFactory.trajectoryCmd("MoveForward.traj"));
    Command niceAndLongPath =
        autoFactory
            .resetOdometry("NiceAndLongPath.traj")
            .andThen(autoFactory.trajectoryCmd("NiceAndLongPath.traj"));

    // paths with marker
    autoRoutine = autoFactory.newRoutine("MoveForwardStop.traj");
    AutoTrajectory moveForwardStopTraj = autoRoutine.trajectory("MoveForwardStop.traj");

    autoRoutine
        .active()
        .onTrue(moveForwardStopTraj.resetOdometry().andThen(moveForwardStopTraj.cmd()));
    moveForwardStopTraj
        .atTime("waitPlease")
        .onTrue(new InstantCommand(() -> DogLog.log("reached marker", true)));

    Command moveForwardStop = autoRoutine.cmd();

    autoChooser.addCmd("redClimb", () -> redClimb);
    autoChooser.addCmd("redDepot", () -> redDepot);
    autoChooser.addCmd("redOutpost", () -> redOutpost);
    autoChooser.addCmd("moveForward", () -> moveForward);
    autoChooser.addCmd("niceLongPath", () -> niceAndLongPath);

    autoChooser.addCmd("moveForwardStop", () -> moveForwardStop);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    Trigger leftTrigger = joystick.leftTrigger();
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction =
            () ->
                leftTrigger.getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast

    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            (BooleanSupplier) (() -> joystick.leftTrigger().getAsBoolean()),
            redside,
            (BooleanSupplier)
                (() -> joystick.rightTrigger().getAsBoolean()), // must be same as shoot cmd binding
            drivetrain);

    drivetrain.setDefaultCommand(swerveJoystickCommand);

    if (Constants.shooterOnRobot && Constants.hopperOnRobot) {
      joystick.rightBumper().onTrue(new WarmUpAndShoot(lebron, hopperSubsystem));
    }

    if (Constants.climberOnRobot) {
      joystick.povUp().onTrue(new L3Climb(climberSubsystem, drivetrain));
      joystick.povRight().onTrue(new L2Climb(climberSubsystem, drivetrain));
      joystick.povDown().onTrue(new L1Climb(climberSubsystem, drivetrain));
    }

    if (Constants.shooterOnRobot) {
      lebron.setDefaultCommand(Commands.run(lebron::stopShooter, lebron));
      joystick.rightTrigger().whileTrue(new Shoot(drivetrain, lebron, hopperSubsystem, redside));
      joystick.x().whileTrue(lebron.shootAtSpeedCommand());
    }

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // INTAKE COMMANDS
    // left trigger -> run intake
    if (Constants.intakeOnRobot) {
      debugJoystick
          .leftTrigger()
          .whileTrue(intakeSubsystem.runRollersCommand(Constants.Intake.Rollers.TARGET_ROLLER_RPS));

      // left trigger + x -> arm to initial pos (0)
      debugJoystick
          .leftTrigger()
          .and(joystick.x())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));

      // left trigger + a -> arm to extended pos (15)
      debugJoystick
          .leftTrigger()
          .and(joystick.a())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED));

      // left trigger + b -> arm to idle pos (45)
      debugJoystick
          .leftTrigger()
          .and(joystick.b())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE));
      intakeSubsystem.setDefaultCommand(
          new ConditionalCommand(
              intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED),
              intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE),
              hopperSubsystem::isHopperSufficientlyEmpty));
    }

    // Auto sequence: choreo forward
    Command trajCommand =
        autoFactory
            .resetOdometry("MoveForward.traj")
            .andThen(autoFactory.trajectoryCmd("MoveForward.traj"));

    // joystick.x().whileTrue(trajCommand);

    if (Constants.hopperOnRobot) {
      debugJoystick
          .rightTrigger()
          .whileTrue(
              hopperSubsystem.runHopperCommand(
                  Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND));
    }

    if (Constants.shooterOnRobot) {
      debugJoystick.leftBumper().whileTrue(lebron.shootAtSpeedCommand());
    }

    // debug these
    if (Constants.climberOnRobot) {
      // L1 commands
      debugJoystick
          .povDown()
          .and(debugJoystick.a())
          .onTrue(climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS));
      debugJoystick
          .povDown()
          .and(debugJoystick.b())
          .onTrue(
              climberSubsystem.MuscleUpCommand(Constants.Climber.MuscleUp.L1_MUSCLE_UP_FORWARD));
      // L2 Commands
      debugJoystick
          .povRight()
          .and(debugJoystick.a())
          .onTrue(climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L2_REACH_POS));
      debugJoystick
          .povRight()
          .and(debugJoystick.b())
          .onTrue(
              climberSubsystem.MuscleUpCommand(Constants.Climber.MuscleUp.L2_MUSCLE_UP_FORWARD));
      // L3 Commands
      debugJoystick
          .povUp()
          .and(debugJoystick.a())
          .onTrue(climberSubsystem.PullUpCommand(Constants.Climber.PullUp.L3_REACH_POS));
      debugJoystick
          .povUp()
          .and(debugJoystick.b())
          .onTrue(
              climberSubsystem.MuscleUpCommand(Constants.Climber.MuscleUp.L3_MUSCLE_UP_FORWARD));
      debugJoystick
          .povUp()
          .onTrue(climberSubsystem.SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE));
      debugJoystick
          .povDown()
          .onTrue(climberSubsystem.SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
