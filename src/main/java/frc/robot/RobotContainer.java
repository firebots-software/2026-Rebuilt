// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commandGroups.ReverseIntakeAndHopper;
import frc.robot.commandGroups.ShootBasicRetract;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.commands.ZeroMuscleUp;
import frc.robot.commands.ZeroPullUp;
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
  // /* Setting up bindings for necessary control of the swerve drive platform */
  // private final SwerveRequest.FieldCentric drive =
  // new SwerveRequest.FieldCentric()
  // .withDeadband(MaxSpeed * 0.1)
  // .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  // .withDriveRequestType(
  // DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  public DoubleSubscriber interMapSpeed = DogLog.tunable("Subsystems/Shooter/Speed", 71.0);
  private BooleanSupplier redside = () -> setAlliance();

  private Field2d field = new Field2d();

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController debugJoystick = new CommandXboxController(1);
  private final CommandXboxController joystick2 = new CommandXboxController(2);
  private final CommandXboxController ronaldoJoystick = new CommandXboxController(3);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final ClimberSubsystem climberSubsystem =
      Constants.climberOnRobot ? new ClimberSubsystem() : null;
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

  private DoubleEntry shooterSpeedEntry;
  private DoubleTopic shooterSpeedTopic;

  public RobotContainer() {
    autoRoutines =
        new AutoRoutines(
            intakeSubsystem, lebron, hopperSubsystem, drivetrain, climberSubsystem, redside);
    autoChooser = autoRoutines.getAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    var table = NetworkTableInstance.getDefault().getTable("Shooter");
    shooterSpeedTopic = table.getDoubleTopic("TargetSpeed");
    shooterSpeedEntry = shooterSpeedTopic.getEntry(60.0);
    shooterSpeedTopic.setPersistent(true);

    // Publish Field2d to SmartDashboard once during initialization
    SmartDashboard.putData("Elastic/Field2d", field);

    configureBindings();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  private void configureBindings() {
    // SWERVE COMMANDS
    joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    DoubleSupplier frontBackFunction = () -> -joystick.getLeftY(),
        leftRightFunction = () -> -joystick.getLeftX(),
        rotationFunction = () -> -joystick.getRightX(),
        speedFunction =
            () ->
                joystick.rightTrigger().getAsBoolean()
                    ? 0d
                    : 1d; // slowmode when left shoulder is pressed, otherwise fast

    SwerveJoystickCommand swerveJoystickCommand =
        new SwerveJoystickCommand(
            frontBackFunction,
            leftRightFunction,
            rotationFunction,
            speedFunction, // slowmode when left shoulder is pressed, otherwise fast
            () -> false,
            (() -> joystick.a().getAsBoolean()),
            redside,
            drivetrain);

    drivetrain.setDefaultCommand(swerveJoystickCommand);
    hopperSubsystem.setDefaultCommand(Commands.run(hopperSubsystem::stop, hopperSubsystem));

    // INTAKE COMMANDS

    // // left bumper -> run intake (changed to a cause mahesh wanted to)
    joystick.leftTrigger().whileTrue(intakeSubsystem.intakeUntilInterruptedCommand());

    // intake default command - stop rollers
    intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeDefault());
    // Commands.runOnce(intakeSubsystem::stopRollers, intakeSubsystem));

    lebron.setDefaultCommand(Commands.run(lebron::stopShooter, lebron));

    joystick
        .rightTrigger()
        .whileTrue(
            new ShootBasicRetract(
                () ->
                    // MiscUtils.computeShootingSpeed(MiscUtils.getDistanceToHub(redside,
                    // drivetrain)),
                    lebron.grabTargetShootingSpeed(MiscUtils.getDistanceToHub(redside, drivetrain)),
                () -> true,
                lebron,
                intakeSubsystem,
                hopperSubsystem));

    if (Constants.Shooter.INTERMAP_TESTING) {
      //   joystick
      //       .a()
      //       .whileTrue(
      //           new ShootBasicRetract(
      //               interMapSpeed, () -> true, lebron, intakeSubsystem, hopperSubsystem));
      // } else {
      //   joystick
      //       .a()
      //       .whileTrue(
      //           new ShootBasicRetract(
      //               () -> 71.0, () -> true, lebron, intakeSubsystem, hopperSubsystem));

      joystick
          .b()
          .whileTrue(
              new ShootBasicRetract(
                  () -> 85.0, () -> true, lebron, intakeSubsystem, hopperSubsystem));

      joystick
          .y()
          .whileTrue(
              new ShootBasicRetract(
                  () -> 100.0, () -> true, lebron, intakeSubsystem, hopperSubsystem));
    }

    joystick2.a().whileTrue(climberSubsystem.movePullUpUpWithVoltageCommand());
    joystick2.b().whileTrue(climberSubsystem.PullUpToCertainPositionCommand(0.1));
    joystick2.x().onTrue(new ZeroPullUp(climberSubsystem));

    joystick2.rightTrigger().and(joystick2.a()).whileTrue(climberSubsystem.moveMuscleUpOutCommand());
    joystick2.rightTrigger().and(joystick.b()).whileTrue(climberSubsystem.moveMuscleUpInCommand());
    joystick2.rightTrigger().and(joystick.x()).whileTrue(new ZeroMuscleUp(climberSubsystem));

    // ronaldoJoystick.a().whileTrue(new ReverseIntakeAndHopper(intakeSubsystem,
    // hopperSubsystem));

    // joystick.a().whileTrue(new ShootBasic(() -> 90.00, () -> lebron.isAtSpeed(),
    // lebron,
    // intakeSubsystem, hopperSubsystem));

    // joystick
    // .rightTrigger()
    // .whileTrue(
    // new ArcAroundAndShoot(
    // drivetrain,`
    // lebron,
    // intakeSubsystem,
    // hopperSubsystem,
    // leftRightFunction,
    // redside,
    // joystick));

    // drivetrain.registerTelemetry(logger::telemeterize);
    // joystick.a().whileTrue(new BumpDTP(drivetrain, () -> true));
  }

  public void visionPeriodic() {

    VisionUtils.visionPeriodic(
        visionFrontRight, visionFrontLeft, visionRearRight, visionRearLeft, drivetrain);

    VisionUtils.fuelGaugeLogs(visionFuelGauge);
  }

  public static boolean setAlliance() {
    return (DriverStation.getAlliance().isEmpty())
        ? false
        : (DriverStation.getAlliance().get() == Alliance.Red);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
