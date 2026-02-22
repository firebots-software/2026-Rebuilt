// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
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
import frc.robot.Constants.Vision.VisionCamera;
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

  private final AutoFactory autoFactory;

  private final AutoRoutines autoRoutines;

  private final AutoChooser autoChooser = new AutoChooser();

  public final VisionSubsystem visionFrontRight =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_RIGHT_CAM)
          : null;
  public final VisionSubsystem visionFrontLeft =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_LEFT_CAM)
          : null;
  // public final VisionSubsystem visionRearRight =
  // Constants.visionOnRobot ? new
  // VisionSubsystem(Constants.Vision.Cameras.REAR_RIGHT_CAM) : null;
  // public final VisionSubsystem visionRearLeft =
  // Constants.visionOnRobot ? new
  // VisionSubsystem(Constants.Vision.Cameras.REAR_LEFT_CAM) : null;

  public final FuelGaugeDetection visionFuelGauge =
      Constants.visionOnRobot
          ? new FuelGaugeDetection(Constants.FuelGaugeDetection.FuelGaugeCamera.FUEL_GAUGE_CAM)
          : null;

  public RobotContainer() {
    // paths without marker
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
      // // left bumper -> run intake
      joystick.leftBumper().whileTrue(intakeSubsystem.intakeUntilInterruptedCommand());

      // intake default command - retract arm if hopper is empty, idle if not
      if (Constants.hopperOnRobot && Constants.visionOnRobot) {
        intakeSubsystem.setDefaultCommand(
            new ConditionalCommand(
                intakeSubsystem.setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_RETRACTED),
                intakeSubsystem.setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_IDLE),
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
          .whileTrue(
              intakeSubsystem.runRollersUntilInterruptedCommand(
                  Constants.Intake.Rollers.TARGET_ROLLER_RPS));

      // left trigger + x -> arm to retracted pos (90)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.x())
          .onTrue(intakeSubsystem.setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_RETRACTED));

      // left trigger + a -> arm to extended pos (15)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.a())
          .onTrue(intakeSubsystem.setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_EXTENDED));

      // left trigger + b -> arm to idle pos (45)
      debugJoystick
          .leftTrigger()
          .and(debugJoystick.b())
          .onTrue(intakeSubsystem.setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_IDLE));
    }

    if (Constants.climberOnRobot) {
      // y -> initiate climb
      // TODO: verify that command is correct
      BooleanSupplier leftside = () -> true; // TODO: get button
      Pose2d poseToDriveTo = new Pose2d();
      if (redside.getAsBoolean()) {
        if (leftside.getAsBoolean()) {
          poseToDriveTo = Constants.Landmarks.RED_TOWER_L;
        } else {
          poseToDriveTo = Constants.Landmarks.RED_TOWER_R;
        }
      } else {
        if (leftside.getAsBoolean()) {
          poseToDriveTo = Constants.Landmarks.BLUE_TOWER_L;
        } else {
          poseToDriveTo = Constants.Landmarks.BLUE_TOWER_R;
        }
      }
      joystick.y().whileTrue(new L3Climb(climberSubsystem, drivetrain, poseToDriveTo));

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
    if (!Constants.visionOnRobot || visionFrontRight == null || visionFrontLeft == null
    /*|| visionRearRight == null
    || visionRearLeft == null */ ) return;

    VisionSubsystem visionFallback;

    VisionCamera fallbackCamera = Constants.Vision.FALLBACK_CAMERA;

    if (fallbackCamera == VisionCamera.FRONT_RIGHT_CAM) visionFallback = visionFrontRight;
    else visionFallback = visionFrontLeft;
    // if (fallbackCamera == VisionCamera.REAR_RIGHT_CAM) visionFallback = visionRearRight;
    // if (fallbackCamera == VisionCamera.REAR_LEFT_CAM) visionFallback = visionRearLeft;

    visionFrontRight.calculateFilteredPose(drivetrain);
    visionFrontLeft.calculateFilteredPose(drivetrain);
    // visionRearRight.calculateFilteredPose(drivetrain);
    // visionRearLeft.calculateFilteredPose(drivetrain);

    VisionSubsystem preferredVision = visionFallback;

    if (!Constants.Vision.SKIP_TO_FALLBACK) {

      double preferredDistance = Double.MAX_VALUE;
      double frontRightDist, frontLeftDist /*, rearRightDist, rearLeftDist */;

      switch (Constants.Vision.CAMERA_SELECTION_METHOD) {
        case MIN:
        default:
          frontRightDist = visionFrontRight.getMinDistance();
          frontLeftDist = visionFrontLeft.getMinDistance();
          // rearRightDist = visionRearRight.getMinDistance();
          // rearLeftDist = visionRearLeft.getMinDistance();
          break;
        case AVG:
          frontRightDist = visionFrontRight.getAverageDistance();
          frontLeftDist = visionFrontLeft.getAverageDistance();
          // rearRightDist = visionRearRight.getAverageDistance();
          // rearLeftDist = visionRearLeft.getAverageDistance();
          break;
        case MAX:
          frontRightDist = visionFrontRight.getMaxDistance();
          frontLeftDist = visionFrontLeft.getMaxDistance();
          // rearRightDist = visionRearRight.getMaxDistance();
          // rearLeftDist = visionRearLeft.getMaxDistance();
          break;
      }

      if (frontRightDist < preferredDistance && visionFrontRight.hasValidMeasurement()) {
        preferredVision = visionFrontRight;
        preferredDistance = frontRightDist;
      }

      if (frontLeftDist < preferredDistance && visionFrontLeft.hasValidMeasurement()) {
        preferredVision = visionFrontLeft;
        preferredDistance = frontLeftDist;
      }

      // if (rearRightDist < preferredDistance &&
      // visionRearRight.hasValidMeasurement()) {
      //   preferredVision = visionRearRight;
      //   preferredDistance = rearRightDist;
      // }

      // if (rearLeftDist < preferredDistance &&
      // visionRearLeft.hasValidMeasurement()) {
      //   preferredVision = visionRearLeft;
      //   preferredDistance = rearLeftDist;
      // }
    }

    if (preferredVision == null) return;

    DogLog.log("Subsystems/Vision/PreferredCamera", preferredVision.getCamera().getLoggingName());

    preferredVision.addFilteredPose(drivetrain);

    DogLog.log("Subsystems/Vision/CompletePoseEstimate", drivetrain.getState().Pose);
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
