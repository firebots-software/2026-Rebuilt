// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
// * KEEP FOR WIN COMMAND TESTING
// import org.wpilib.math.geometry.Pose2d;
// import org.wpilib.math.geometry.Rotation2d;
// import org.wpilib.math.geometry.Translation2d;
import frc.robot.Constants.Swerve.Auto.AutoList;
// * KEEP FOR WIN COMMAND TESTING
import frc.robot.commandGroups.ShootCommandGroups.ShootPassing;
import frc.robot.commandGroups.ShootCommandGroups.ShootWithAim;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelGaugeDetection;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeVisionDetection;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.CustomController;
import frc.robot.util.MiscUtils;
import frc.robot.util.Targeting;
import frc.robot.util.VisionUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.tunable.Tunables;

public class RobotContainer {
  private BooleanSupplier redside = RobotContainer::isRedAlliance;

  //   private Field2d field = new Field2d();
  private final CommandGamepad joystick = new CommandGamepad(0);
  private final CustomController secondController =
      Constants.secondControllerConnected ? new CustomController(4) : null;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem(drivetrain, redside) : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron =
      Constants.shooterOnRobot ? new ShooterSubsystem(drivetrain, redside) : null;

  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;

  public final VisionSubsystem visionFrontRight =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_RIGHT_CAM, drivetrain)
          : null;
  public final VisionSubsystem visionFrontLeft =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_LEFT_CAM, drivetrain)
          : null;
  public final VisionSubsystem visionRearRight =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.REAR_RIGHT_CAM, drivetrain)
          : null;
  public final VisionSubsystem visionRearLeft =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.REAR_LEFT_CAM, drivetrain)
          : null;

  public final FuelGaugeDetection visionFuelGauge =
      Constants.fuelGaugeOnRobot
          ? new FuelGaugeDetection(Constants.FuelGaugeDetection.FuelGaugeCamera.FUEL_GAUGE_CAM)
          : null;
  public final IntakeVisionDetection visionIntake =
      Constants.intakeVisionOnRobot
          ? new IntakeVisionDetection(Constants.IntakeVision.IntakeVisionCamera.INTAKE_CAM)
          : null;

  public final LEDSubsystem leds =
      new LEDSubsystem(
          () -> MiscUtils.areWeActive(2.0),
          () ->
              Targeting.distMeters(drivetrain, Targeting.getHub(redside)) < 4.47
                  && inAllianceSide());

  // * KEEP FOR INTERMAP TESTING
  //   private double hoodAngle = 18.369;
  //   private double shooterSpeed = 58.0;

  public RobotContainer() {
    autoRoutines = new AutoRoutines(intakeSubsystem, lebron, hopperSubsystem, drivetrain, redside);
    autoChooser = autoRoutines.getAutoChooser();
    // Tunables.publish("Auto", autoChooser);
    configureBindings();
  }

  //   public void doTelemetry() {
  //     logger.telemeterize(drivetrain.getCurrentState());

  //     String commandName = "nah";

  //     if (drivetrain.getCurrentCommand() != null) {
  //       commandName = drivetrain.getCurrentCommand().getName();
  //     }
  //     DogLog.log("Robot/SwerveDriveCommand", commandName);
  //   }

  //   public CommandSwerveDrivetrain getDrivetrain() {
  //     return drivetrain;
  //   }

  private void configureBindings() {
    // Swerve
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY();
    DoubleSupplier leftRightFunction = () -> -joystick.getLeftX();
    DoubleSupplier rotationFunction = () -> -joystick.getRightX();
    DoubleSupplier speedFunction = () -> 1d;

    SwerveJoystickCommand swerveJoystickDefaultCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction,
            () -> true,
            joystick.leftTrigger()::getAsBoolean,
            () -> false,
            redside,
            drivetrain,
            () -> false,
            () -> false);

    joystick.faceLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.setDefaultCommand(swerveJoystickDefaultCommand);

    // Intake
    intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeDefault());
    joystick.leftBumper().whileTrue(intakeSubsystem.intakeUntilInterruptedCommand());

    joystick
        .faceRight()
        .whileTrue(
            intakeSubsystem
                .outtakeUntilInterruptedCommand()
                .alongWith(
                    hopperSubsystem.runHopperUntilInterruptedCommand(
                        -Constants.Hopper.TARGET_SURFACE_SPEED_MPS)));

    // * KEEP FOR WIN COMMAND
    // joystick
    //     .a()
    //     .whileTrue(
    //         new DriveToAndShoot(
    //             () -> (new Pose2d(new Translation2d(3.0, 5.0), new Rotation2d())),
    //             lebron,
    //             intakeSubsystem,
    //             hopperSubsystem,
    //             drivetrain,
    //             redside));

    // joystick
    //     .a()
    //     .whileTrue(
    //         new DriveToPose(
    //             drivetrain,
    //             () ->
    //                 new Pose2d(
    //                     new Translation2d(2.462480068206787, 2.26101016998291), new
    // Rotation2d())));

    // * KEEP FOR INTERMAP TESTING
    // joystick
    //     .rightTrigger()
    //     .whileTrue(
    //         lebron
    //             .shootAtSpeedHoodCommand(() -> shooterSpeed, () -> hoodAngle)
    //             .alongWith(Commands.waitUntil(lebron::isAtSpeed).andThen
    //
    // (hopperSubsystem.runHopperUntilInterruptedCommand().alongWith(Commands.waitSeconds(0.4).andThen(intakeSubsystem.powerRetractRollersCommand())))));

    if (Constants.secondControllerConnected) {
      secondController.intakeOverride().whileTrue(intakeSubsystem.retractIntakeCommand());
    }

    // Hopper
    hopperSubsystem.setDefaultCommand(hopperSubsystem.run(hopperSubsystem::stop));

    // Shooter
    lebron.setDefaultCommand(lebron.runOnce(lebron::stopShooter));
    joystick
        .rightTrigger()
        .whileTrue(
            new ShootWithAim(
                frontBackFunction,
                leftRightFunction,
                lebron,
                intakeSubsystem,
                hopperSubsystem,
                drivetrain,
                redside,
                Constants.secondControllerConnected
                    ? secondController.visionShootingLockout()
                    : () -> false));

    joystick
        .rightBumper()
        .whileTrue(
            new ShootPassing(
                frontBackFunction,
                leftRightFunction,
                lebron,
                intakeSubsystem,
                hopperSubsystem,
                drivetrain,
                redside));

    if (Constants.secondControllerConnected) {
      secondController.reverseShoot().whileTrue(lebron.shootAtSpeedCommand(-45.0));
    }

    // * KEEP FOR INTERMAP TESTING
    // joystick.x().onTrue(new InstantCommand(() -> hoodAngle+=0.2));
    // joystick.y().onTrue(new InstantCommand(() -> hoodAngle-=0.2));
    // joystick.a().onTrue(new InstantCommand(() -> shooterSpeed+=0.5));
    // joystick.b().onTrue(new InstantCommand(() -> shooterSpeed-=0.5));
  }

  public static boolean isRedAlliance() {
    return MatchState.getAlliance().isEmpty()
        ? false
        : MatchState.getAlliance().get() == Alliance.RED;
  }

  public void visionPeriodic() {
    VisionUtils.visionPeriodic(
        visionFrontRight, visionFrontLeft, visionRearRight, visionRearLeft, drivetrain);
    leds.visionStatusIndicators(visionFrontLeft, visionFrontRight, visionRearLeft, visionRearRight);
  }

  public boolean inAllianceSide() {
    return redside.getAsBoolean()
        ? drivetrain.getPose().getX() > Constants.Landmarks.RED_HUB.getX()
        : drivetrain.getPose().getX() < Constants.Landmarks.BLUE_HUB.getX();
  }

  public Command getAutonomousCommand() {
    DogLog.log("Robot/selectedAuto", autoChooser.selectedCommand().toString());
    return autoChooser.selectedCommand();

  }

  public Command getAutonomousCommand(AutoList auto) {
    String selected = autoChooser.select(auto.getInternalName());
    DogLog.log("Robot/selectedAuto", selected);
    // if (!selected.equals(auto.getInternalName())) return null;
    return autoChooser.selectedCommand();
  }
}
