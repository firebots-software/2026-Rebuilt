// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.ShootCommandGroups.ShootWithAim;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.SwerveCommands.SwerveJoystickDefaultCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FuelGaugeDetection;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeVisionDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.CustomController;
import frc.robot.util.VisionUtils;
import frc.robot.util.VisionUtils.IntakeVisionTarget;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private BooleanSupplier redside = RobotContainer::isRedAlliance;

  private Field2d field = new Field2d();

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CustomController secondController = new CustomController(4);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem() : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron =
      Constants.shooterOnRobot ? new ShooterSubsystem(drivetrain, redside) : null;

  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;

  public final VisionSubsystem visionFrontRight =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_RIGHT_CAM)
          : null;
  public final VisionSubsystem visionFrontLeft =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.FRONT_LEFT_CAM)
          : null;
  public final VisionSubsystem visionRearRight =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.REAR_RIGHT_CAM)
          : null;
  public final VisionSubsystem visionRearLeft =
      Constants.visionOnRobot
          ? new VisionSubsystem(Constants.Vision.VisionCamera.REAR_LEFT_CAM)
          : null;

  public final FuelGaugeDetection visionFuelGauge =
      Constants.fuelGaugeOnRobot
          ? new FuelGaugeDetection(Constants.FuelGaugeDetection.FuelGaugeCamera.FUEL_GAUGE_CAM)
          : null;
  public final IntakeVisionDetection visionIntake =
      Constants.intakeVisionOnRobot
          ? new IntakeVisionDetection(Constants.IntakeVision.IntakeVisionCamera.INTAKE_CAM)
          : null;

  public RobotContainer() {
    autoRoutines = new AutoRoutines(intakeSubsystem, lebron, hopperSubsystem, drivetrain, redside);
    autoChooser = autoRoutines.getAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Elastic/Field2d", field);

    configureBindings();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  private void configureBindings() {
    // Swerve
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY();
    DoubleSupplier leftRightFunction = () -> -joystick.getLeftX();
    DoubleSupplier rotationFunction = () -> -joystick.getRightX();
    DoubleSupplier speedFunction = () -> 1d;
    SwerveJoystickDefaultCommand swerveJoystickDefaultCommand =
        new SwerveJoystickDefaultCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction,
            () -> true,
            joystick.leftTrigger()::getAsBoolean,
            redside,
            drivetrain,
            visionIntake,
            joystick.leftBumper()::getAsBoolean,
            secondController.intakeVisionLockout(),
            intakeSubsystem::atExtendedPosition);

    joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.setDefaultCommand(swerveJoystickDefaultCommand);

    // Intake
    intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeDefault());
    joystick.leftBumper().whileTrue(intakeSubsystem.intakeUntilInterruptedCommand());
    joystick
        .a()
        .whileTrue(
            intakeSubsystem
                .outtakeUntilInterruptedCommand()
                .alongWith(
                    hopperSubsystem.runHopperUntilInterruptedCommand(
                        -Constants.Hopper.TARGET_SURFACE_SPEED_MPS)));
    secondController.intakeOverride().whileTrue(intakeSubsystem.retractIntakeCommand());

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
                secondController.visionShootingLockout()));
    secondController.reverseShoot().whileTrue(lebron.shootAtSpeedCommand(-45.0));
  }

  public void visionPeriodic() {
    VisionUtils.visionPeriodic(
        visionFrontRight, visionFrontLeft, visionRearRight, visionRearLeft, drivetrain);

    visionFuelGauge.periodic();
    VisionUtils.fuelGaugeLogs(visionFuelGauge);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isEmpty()
        ? false
        : DriverStation.getAlliance().get() == Alliance.Red;
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
