package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.MathUtils.Vector3;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveJoystickCommandInArc extends Command {
  protected final DoubleSupplier tangentSpdFunction, speedControlFunction;

  protected final Supplier<Translation2d> poseToTarget;

  private final Pose2d center;

  protected final BooleanSupplier fieldRelativeFunction;

  protected final CommandSwerveDrivetrain swerveDrivetrain;
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private BooleanSupplier redSide;

  public SwerveJoystickCommandInArc(
      Pose2d center,
      DoubleSupplier tangentSpeedFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      Supplier<Translation2d> poseToTarget,
      CommandSwerveDrivetrain swerveSubsystem) {
    this.center = center;
    this.tangentSpdFunction = tangentSpeedFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.speedControlFunction = speedControlFunction;
    this.poseToTarget = poseToTarget;
    this.swerveDrivetrain = swerveSubsystem;
    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double thetaFromCenter =
        Math.atan2(
            Vector3.subtract(new Vector3(swerveDrivetrain.getState().Pose), new Vector3(center)).y,
            Vector3.subtract(new Vector3(swerveDrivetrain.getState().Pose), new Vector3(center)).x);
    double tangentialSpeed = tangentSpdFunction.getAsDouble();
    tangentialSpeed =
        Math.abs(tangentialSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? tangentialSpeed : 0.0;

    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_PERCENT_SPEED_RANGE * (speedControlFunction.getAsDouble()))
            + Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT;

    double xSpeed = tangentialSpeed * Math.cos(thetaFromCenter + Math.PI / 2f);
    double ySpeed = tangentialSpeed * Math.sin(thetaFromCenter + Math.PI / 2f);

    double magnitude = Math.hypot(xSpeed, ySpeed);

    if (magnitude < Constants.OI.LEFT_JOYSTICK_DEADBAND) {
        xSpeed = 0.0;
        ySpeed = 0.0;
    } else {
        double xDir = xSpeed / magnitude;
        double yDir = ySpeed / magnitude;

        magnitude = (magnitude - Constants.OI.LEFT_JOYSTICK_DEADBAND) / (1.0 - Constants.OI.LEFT_JOYSTICK_DEADBAND);

        double magnitudeSquared = magnitude * magnitude;
        magnitudeSquared *= Constants.Swerve.GLOBAL_SWERVE_MULT;

        xSpeed = xDir * magnitudeSquared;
        ySpeed = yDir * magnitudeSquared;
    }

    xSpeed = xSpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed = ySpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    DogLog.log("Commands/arcjoystickCommand/xSpeed", xSpeed);
    DogLog.log("Commands/arcjoystickCommand/ySpeed", ySpeed);
    DogLog.log("Information/fieldCentric", fieldRelativeFunction.getAsBoolean());
    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    double turn = swerveDrivetrain.calculateRequiredRotationalRateWithFF(poseToTarget.get());

    DogLog.log("Commands/arcjoystickCommand/turnReq", turn);

    SwerveRequest drive =
        !fieldRelativeFunction.getAsBoolean()
            ? fieldCentricDrive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(turn)
            : robotCentricDrive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(turn);

    // Applies request
    this.swerveDrivetrain.setControl(drive);
  } // Drive counterclockwise with negative X (left))

  @Override
  public void end(boolean interrupted) {
    // Applies SwerveDriveBrake (brakes the robot by turning wheels)
    this.swerveDrivetrain.setControl(new SwerveRequest.Idle());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
