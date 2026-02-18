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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.ClimbCommands.L3Climb;
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

  private final AutoFactory autoFactory; // no marker

  public final AutoRoutine autoRoutine; // with markers

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
            speedFunction,
            joystick.leftTrigger()::getAsBoolean,
            redside,
            joystick.rightTrigger()::getAsBoolean, // must be same as shoot cmd binding
            drivetrain);

    drivetrain.setDefaultCommand(swerveJoystickCommand);

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
      // shooter default command - stop shooter
      lebron.setDefaultCommand(lebron.run(lebron::stopShooter));
    }

    if (Constants.climberOnRobot) {
      // y -> initiate climb
      // TODO: verify that command is correct
      joystick.y().whileTrue(new L3Climb(climberSubsystem, drivetrain));

      // a -> zero climber
      joystick.a().onTrue(climberSubsystem.runOnce(climberSubsystem::resetPullUpPositionToZero));
    }

    // TODO: left trigger -> run LockOnCommand (not yet defined)
    // joystick.leftTrigger().whileTrue(new LockOnCommand(....));

    // TODO: right trigger -> shoot + arc lock (arc lock not yet defined)
    // verify command is correct and that sequential is correct type of commandgroup
    // joystick.rightTrigger().whileTrue(new SequentialCommandGroup(
    //    new Shoot(drivetrain, lebron, hopperSubsystem, redside),
    //    new ArcLock(.....)
    // ));

    // Auto sequence: choreo forward
    Command trajCommand =
        autoFactory
            .resetOdometry("MoveForward.traj")
            .andThen(autoFactory.trajectoryCmd("MoveForward.traj"));

    // joystick.x().whileTrue(trajCommand);

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
