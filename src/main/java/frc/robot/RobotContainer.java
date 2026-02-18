// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
import frc.robot.subsystems.FuelGaugeDetection;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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

  //   private final AutoFactory autoFactory; // no marker

  //   public final AutoRoutine autoRoutine; // with markers

  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;

  private final AutoChooser autoChooser = new AutoChooser();

  public final VisionSubsystem visionRight =
      Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.RIGHT_CAM) : null;
  public final VisionSubsystem visionLeft =
      Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.LEFT_CAM) : null;
  // public final VisionSubsystem visionRearRight =
  // Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.REAR_RIGHT_CAM) : null;
  // public final VisionSubsystem visionRearLeft =
  // Constants.visionOnRobot ? new VisionSubsystem(Constants.Vision.Cameras.REAR_LEFT_CAM) : null;
  public final FuelGaugeDetection visionFuelGauge =
      Constants.visionOnRobot ? new FuelGaugeDetection(Constants.Vision.Cameras.COLOR_CAM) : null;

  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines =
        new AutoRoutines(
            autoFactory, intakeSubsystem, lebron, hopperSubsystem, drivetrain, climberSubsystem);

    // Add all autos here
    autoChooser.addCmd(
        "Pedri - Left Side",
        () ->
            autoRoutines.Pedri(
                null, "LeftIntakeL", "LeftShootPositioning", "LeftClimbPositioning"));
    autoChooser.addCmd("Trial Path", () -> autoRoutines.trialPath());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
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
    // right bumper -> run intake
    if (Constants.intakeOnRobot) {
      joystick.x().whileTrue(intakeSubsystem.armToDegrees(35.0));
      joystick.y().whileTrue(intakeSubsystem.armToDegrees(0.0));
      intakeSubsystem.setDefaultCommand(
          new ConditionalCommand(
              intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED),
              intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE),
              () -> hopperSubsystem.isHopperSufficientlyEmpty(visionFuelGauge)));

      // left trigger + x -> arm to initial pos (0)
      //   joystick
      //       .leftTrigger()
      //       .and(joystick.x())
      //       .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));

      // left trigger + a -> arm to extended pos (15)
      //   joystick
      //       .leftTrigger()
      //       .and(joystick.a())
      //       .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED));

      //   // left trigger + b -> arm to idle pos (45)
      //   joystick
      //       .leftTrigger()
      //       .and(joystick.b())
      //       .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE));

      //   // left trigger + y -> arm to retracted pos (90)
      //   joystick
      //       .leftTrigger()
      //       .and(joystick.y())
      //       .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
    }

    if (Constants.hopperOnRobot) {
      //   joystick.x().whileTrue(hopperSubsystem.runHopperCommand(4.0));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void visionPeriodic() {
    if (!Constants.visionOnRobot || visionRight == null || visionLeft == null
    /*|| visionRearRight == null
    || visionRearLeft == null */ ) return;
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
