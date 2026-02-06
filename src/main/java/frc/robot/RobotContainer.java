// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MiscUtils;
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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ClimberSubsystem climberSubsystem =
      Constants.climberOnRobot ? new ClimberSubsystem() : null;
  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem() : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron = Constants.shooterOnRobot ? new ShooterSubsystem() : null;
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final HopperSubsystem hopper = new HopperSubsystem();

  private final AutoFactory autoFactory;

  private final AutoRoutines autoRoutines;

  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    Command redClimb =
        autoFactory
            .resetOdometry("RedClimb.traj")
            .andThen(autoFactory.trajectoryCmd("RedClimb.traj"));
    Command redDepot =
        autoFactory
            .resetOdometry("RedDepot.traj")
            .andThen(autoFactory.trajectoryCmd("RedClimb.traj"));
    Command redOutpost =
        autoFactory
            .resetOdometry("RedOutpost.traj")
            .andThen(autoFactory.trajectoryCmd("RedClimb.traj"));
    Command moveForward =
        autoFactory
            .resetOdometry("MoveForward.traj")
            .andThen(autoFactory.trajectoryCmd("RedClimb.traj"));

    autoChooser.addCmd("redClimb", () -> redClimb);
    autoChooser.addCmd("redDepot", () -> redDepot);
    autoChooser.addCmd("redOutpost", () -> redOutpost);
    autoChooser.addCmd("moveForward", () -> moveForward);

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

    if (Constants.shooterOnRobot) {
      joystick.rightTrigger().whileTrue(new Shoot(drivetrain, lebron, hopperSubsystem, redside));
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
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // INTAKE COMMANDS
    // right bumper -> run intake
    if (Constants.intakeOnRobot) {
      joystick.rightBumper().whileTrue(intakeSubsystem.runIntake());

      // left trigger + x -> arm to initial pos (0)
      joystick
          .leftTrigger()
          .and(joystick.x())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_INITIAL));

      // left trigger + a -> arm to extended pos (15)
      joystick
          .leftTrigger()
          .and(joystick.a())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED));

      // left trigger + b -> arm to idle pos (45)
      joystick
          .leftTrigger()
          .and(joystick.b())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE));

      // left trigger + y -> arm to retracted pos (90)
      joystick
          .leftTrigger()
          .and(joystick.y())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
    }

    // Auto sequence: choreo forward
    Command trajCommand =
        autoFactory
            .resetOdometry("MoveForward.traj")
            .andThen(autoFactory.trajectoryCmd("MoveForward.traj"));

    joystick.x().whileTrue(trajCommand);

    joystick.x().whileTrue(trajCommand);

    joystick
        .x()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () ->
                    MiscUtils.plus(
                        drivetrain.getCurrentState().Pose, new Translation2d(-5, 0)))); // x is neg

    joystick
        .povDown()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () -> MiscUtils.plus(drivetrain.getCurrentState().Pose, new Translation2d(0, 4))));

    joystick
        .y()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () ->
                    MiscUtils.plusWithRotation(
                        drivetrain.getCurrentState().Pose,
                        new Pose2d(new Translation2d(2, 2), new Rotation2d(1.5708)))));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public static void setAlliance() {
    redAlliance =
        (DriverStation.getAlliance().isEmpty())
            ? false
            : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    return autoShoot();
    // return autoChooser.selectedCommand();
  }

  public Command autoShoot() {
    return new SequentialCommandGroup(
        shooter.ShootAtSpeed(),
        new WaitUntilCommand(() -> shooter.isAtSpeed()),
        hopper.RunHopper(Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC).withTimeout(6.7),
        Commands.runOnce(() -> shooter.stop()));
  }
}
