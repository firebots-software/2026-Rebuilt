// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandGroups.ShootCommands.ShootBasicRetract;
import frc.robot.commandGroups.ShootCommands.ShootWithAim;
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
import frc.robot.util.VisionUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  // Query driver station for alliance data
  private BooleanSupplier redside = RobotContainer::isRedAlliance;

  // Field2d instance for Elastic
  private Field2d field = new Field2d();

  // Joysticks
  private final CommandXboxController mainController = new CommandXboxController(0);

  // Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final ClimberSubsystem climberSubsystem =
      Constants.climberOnRobot ? new ClimberSubsystem() : null;
  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem() : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron =
      Constants.shooterOnRobot ? new ShooterSubsystem(drivetrain, redside) : null;

  // Vision
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

  // Auto
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;

  public RobotContainer() {
    autoRoutines =
        new AutoRoutines(
            intakeSubsystem, lebron, hopperSubsystem, drivetrain, climberSubsystem, redside);
    autoChooser = autoRoutines.getAutoChooser();

    // Publish auto chooser/Field2d data to SmartDashboard so Elastic sees them
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Elastic/Field2d", field);
    // Log red side data for Elastic
    DogLog.log("Elastic/IsRedSide", redside.getAsBoolean());

    configureBindings();
  }

  private void configureBindings() {
    // Swerve
    // x -> zero swerve
    mainController.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    DoubleSupplier frontBackFunction = () -> -mainController.getLeftY();
    DoubleSupplier leftRightFunction = () -> -mainController.getLeftX();
    DoubleSupplier rotationFunction = () -> -mainController.getRightX();
    DoubleSupplier speedFunction = () -> 1d;

    // Default swerve command (use joysticks for translation/rotations)
    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction,
            () -> false,
            mainController.leftTrigger()::getAsBoolean,
            redside,
            drivetrain);
    drivetrain.setDefaultCommand(swerveJoystickCommand);

    // Intake
    // left bumper -> run intake
    mainController.leftBumper().whileTrue(intakeSubsystem.intakeUntilInterruptedCommand());
    // move arm to idle pos and stop rollers when the intake's not in use
    intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeDefault());

    // Hopper
    // stop hopper motors when the hopper's not in use
    hopperSubsystem.setDefaultCommand(hopperSubsystem.run(hopperSubsystem::stop));

    // Shooter
    // stop shooter when the shooter's not in use
    lebron.setDefaultCommand(lebron.shootAtSpeedCommand());

    // right trigger -> run shooter and hopper while power retracting intake
    mainController
        .rightTrigger()
        .whileTrue(
            new ShootBasicRetract(
                // () -> MiscUtils.computeShootingSpeed(...),
                () ->
                    lebron.grabTargetShootingSpeed(MiscUtils.getDistanceToHub(redside, drivetrain)),
                lebron,
                intakeSubsystem,
                hopperSubsystem));

    // right bumper -> same as above but with aiming
    mainController
        .rightBumper()
        .whileTrue(
            new ShootWithAim(
                frontBackFunction,
                leftRightFunction,
                lebron,
                intakeSubsystem,
                hopperSubsystem,
                drivetrain,
                redside));
  }

  public void visionPeriodic() {
    VisionUtils.visionPeriodic(
        visionFrontRight, visionFrontLeft, visionRearRight, visionRearLeft, drivetrain);
    VisionUtils.fuelGaugeLogs(visionFuelGauge);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        ? DriverStation.getAlliance().get() == Alliance.Red
        : false;
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
