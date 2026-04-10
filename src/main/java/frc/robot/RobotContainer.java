package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandGroups.BundtShot;
import frc.robot.commandGroups.Intake;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.OtherXBoxController;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OI
  private final OtherXBoxController joystick =
      new OtherXBoxController(Constants.OI.JOYSTICK_A_PORT);
  // Subsystems
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final PeterSubsystem peterSubsystem = PeterSubsystem.getInstance();
  private final JoystickSubsystem joystickSubsystem = new JoystickSubsystem(joystick.getHID());
  // Logging
  private final Telemetry logger =
      new Telemetry(Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

  public RobotContainer() {
    configureBindings();
  }

  // Starts telemetry operations (essentially logging -> look on SmartDashboard, AdvantageScope)
  public void doTelemetry() {
    logger.telemeterize(driveTrain.getState());
    DogLog.log("drivetrain/heading", driveTrain.getState().RawHeading.getDegrees() * 360);

    double x = driveTrain.getState().Speeds.vxMetersPerSecond;
    double y = driveTrain.getState().Speeds.vyMetersPerSecond;

    double speed = Math.sqrt(x * x + y * y);

    DogLog.log("drivetrain/speed(fps)", speed * 3.2808);
  }

  private void configureBindings() {
    // For outreach, we only want
    // 1. drive
    // 2. intake
    // 3. fire at set angle

    // Driving
    //     * Left joystick = translation
    //     * Right joystick = rotation
    //     * Right shoulder = speed increase
    // Speed is determined by
    // Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT for when right shoulder is not pressed
    // Constants.Swerve.TELE_DRIVE_FAST_MODE_SPEED_PERCENT for when right shoulder is pressed
    // Trigger rightShoulderTrigger = joystick.rightBumper();
    // Supplier<Double> frontBackFunction = () -> -joystick.getLeftY(),
    //     leftRightFunction = () -> -joystick.getLeftX(),
    //     rotationFunction = () -> -joystick.getRightX(),
    //     speedFunction = () -> rightShoulderTrigger.getAsBoolean() ? 1d : 0d;
    // Supplier<Boolean> fieldRelative = () -> false;
    // SwerveJoystickCommand swerveJoystickCommand =
    //     new SwerveJoystickCommand(
    //         frontBackFunction,
    //         leftRightFunction,
    //         rotationFunction,
    //         speedFunction,
    //         fieldRelative,
    //         driveTrain);
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
            () -> false,
            driveTrain);

    driveTrain.setDefaultCommand(swerveJoystickCommand);
    driveTrain.registerTelemetry(logger::telemeterize);

    // Intake - left trigger
    // Angle controlled by Constants.Arm.INTAKE_ANGLE
    joystick.leftTrigger().whileTrue(new Intake(peterSubsystem, armSubsystem, joystickSubsystem));

    // Shoot - right trigger
    // Angle controlled by Constants.Arm.BUNDT_ANGLE
    joystick
        .rightTrigger()
        .whileTrue(new BundtShot(peterSubsystem, armSubsystem, joystickSubsystem));

    // When no Commands are being issued, Peter motors should not be moving
    peterSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> {
              peterSubsystem.stopIntake();
              peterSubsystem.stopLeftShooter();
              peterSubsystem.stopRightShooter();
              peterSubsystem.stopPreShooterMotor();
            },
            peterSubsystem));

    // Outtake - left shoulder
    joystick
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                    new RunCommand(
                        () -> {
                          peterSubsystem.reverseMechanism();
                        },
                        peterSubsystem),
                    ArmToAngleCmd.toNeutral(armSubsystem).withTolerance(1))
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    joystick
        .y()
        .onTrue(
            driveTrain.runOnce(
                () ->
                    driveTrain.resetPose(
                        new Pose2d(driveTrain.getPose().getTranslation(), new Rotation2d(0)))));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(1);
  }
}
