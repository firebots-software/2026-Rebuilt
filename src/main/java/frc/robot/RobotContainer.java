// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.ArcAroundAndShoot;
import frc.robot.commandGroups.ClimbCommands.L3Climb;
import frc.robot.commandGroups.WarmUpAndShoot;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelGaugeDetection;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.MiscUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

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
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final HopperSubsystem hopper = new HopperSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();

  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;

  public final VisionSubsystem visionRight =
      Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.RIGHT_CAM) : null;
  public final VisionSubsystem visionLeft =
      Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.LEFT_CAM) : null;
  // public final VisionSubsystem visionRearRight =
  // Constants.visionOnRobot ? new
  // VisionSubsystem(Constants.Vision.Cameras.REAR_RIGHT_CAM) : null;
  // public final VisionSubsystem visionRearLeft =
  // Constants.visionOnRobot ? new
  // VisionSubsystem(Constants.Vision.Cameras.REAR_LEFT_CAM) : null;
  public final FuelGaugeDetection visionFuelGauge =
      Constants.visionOnRobot ? new FuelGaugeDetection(Constants.Vision.Cameras.COLOR_CAM) : null;

  public RobotContainer() {
    autoRoutines =
        new AutoRoutines(intakeSubsystem, lebron, hopperSubsystem, drivetrain, climberSubsystem);

    autoChooser = autoRoutines.getAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  private void configureBindings() {
    // Swerve bindings - left joystick for translation, right joystick for rotation
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
            () -> joystick.leftTrigger().getAsBoolean(),
            drivetrain);

    drivetrain.setDefaultCommand(swerveJoystickCommand);

    if (Constants.shooterOnRobot && Constants.hopperOnRobot) {
      joystick
          .rightBumper()
          .onTrue(new WarmUpAndShoot(() -> 10d, () -> true, lebron, hopperSubsystem));
    }
    // x -> zero swerve
    joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    if (Constants.intakeOnRobot) {
      // left bumper -> run intake
      joystick.leftBumper().whileTrue(intakeSubsystem.extendArmAndRunRollers());

      // intake default command - retract arm if hopper is empty, idle if not
      if (Constants.hopperOnRobot && Constants.visionOnRobot) {
        intakeSubsystem.setDefaultCommand(
            new ConditionalCommand(
                intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED),
                intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE),
                () -> hopperSubsystem.isHopperSufficientlyEmpty(visionFuelGauge)));
      }
    }

    if (Constants.shooterOnRobot) {
      lebron.setDefaultCommand(Commands.run(lebron::stopShooter, lebron));
      joystick
          .rightTrigger()
          .whileTrue(
              new ArcAroundAndShoot(
                  drivetrain,
                  lebron,
                  intakeSubsystem,
                  hopperSubsystem,
                  leftRightFunction,
                  redside,
                  joystick));
    }

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

    // INTAKE COMMANDS (DEBUG)
    // left trigger -> run intake
    if (Constants.intakeOnRobot) {
      debugJoystick
          .leftTrigger()
          .whileTrue(intakeSubsystem.runRollersCommand(Constants.Intake.Rollers.TARGET_ROLLER_RPS));

      // left trigger + x -> arm to retracted pos (90)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.x())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));

      // left trigger + a -> arm to extended pos (15)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.a())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED));

      // left trigger + b -> arm to idle pos (45)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.b())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE));
    }

    if (Constants.climberOnRobot) {
      // y -> initiate climb
      // TODO: verify that command is correct
      joystick.y().whileTrue(new L3Climb(climberSubsystem, drivetrain));

      // a -> zero climber
      joystick.a().onTrue(climberSubsystem.runOnce(climberSubsystem::resetPullUpPositionToZero));
    }

    // TODO: TURN THESE INTO DEBUG COMMANDS IN THE FUTURE

    // if (Constants.hopperOnRobot) {
    //   // joystick.x().whileTrue(hopperSubsystem.runHopperCommand(4.0));
    // }

    debugJoystick
        .povUp()
        .whileTrue(
            new DriveToPose(
                    drivetrain,
                    () ->
                        MiscUtils.plus(drivetrain.getCurrentState().Pose, new Translation2d(2, 0)))
                .andThen(new InstantCommand(() -> DogLog.log("first dtp done", true)))
                .andThen(
                    new DriveToPose(
                        drivetrain,
                        () ->
                            MiscUtils.plus(
                                drivetrain.getCurrentState().Pose, new Translation2d(0, -2)))));

    debugJoystick
        .povDown()
        .whileTrue(
            new DriveToPose(
                    drivetrain,
                    () ->
                        MiscUtils.plusWithRotation(
                            drivetrain.getCurrentState().Pose,
                            new Pose2d(new Translation2d(2, 0), new Rotation2d(1.5708))))
                .andThen(
                    new DriveToPose(
                        drivetrain,
                        () ->
                            MiscUtils.plusWithRotation(
                                drivetrain.getCurrentState().Pose,
                                new Pose2d(new Translation2d(0, -2), new Rotation2d(1.5708))))));

    debugJoystick
        .povRight()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () ->
                    MiscUtils.plusWithRotation(
                        drivetrain.getCurrentState().Pose,
                        new Pose2d(new Translation2d(2, 0), new Rotation2d(1.5708)))));

    debugJoystick
        .povLeft()
        .whileTrue(
            new DriveToPose(
                drivetrain,
                () -> MiscUtils.plus(drivetrain.getCurrentState().Pose, new Translation2d(2, 0))));

    // TODO: left trigger -> run LockOnCommand (not yet defined)
    // joystick.leftTrigger().whileTrue(new LockOnCommand(....));

    // TODO: right trigger -> shoot + arc lock (arc lock not yet defined)
    // verify command is correct and that sequential is correct type of commandgroup
    // joystick.rightTrigger().whileTrue(new SequentialCommandGroup(
    // new Shoot(drivetrain, lebron, hopperSubsystem, redside),
    // new ArcLock(.....)
    // ));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void visionPeriodic() {
    if (!Constants.visionOnRobot || visionRight == null || visionLeft == null /*
         * || visionRearRight == null
         * || visionRearLeft == null
         */) return;
    visionRight.addFilteredPose(drivetrain);
    visionLeft.addFilteredPose(drivetrain);
    // visionRearRight.addFilteredPose(drivetrain);
    // visionRearLeft.addFilteredPose(drivetrain);
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
