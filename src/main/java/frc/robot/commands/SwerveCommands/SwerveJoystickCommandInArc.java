package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCommandInArc extends Command {
  protected final DoubleSupplier tangentSpdFunction, speedControlFunction, angleToPointTo;

  private final Pose3d centre;

  protected final BooleanSupplier fieldRelativeFunction;

  // Limits rate of change (in this case x, y, and turning movement)
  protected final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  protected final CommandSwerveDrivetrain swerveDrivetrain;
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private BooleanSupplier redSide;

  public SwerveJoystickCommandInArc(
      Pose3d centre,
      DoubleSupplier tangentSpeedFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      DoubleSupplier angleToPointTo,
      CommandSwerveDrivetrain swerveSubsystem,
      BooleanSupplier redside) {
    this.centre = centre;
    this.tangentSpdFunction = tangentSpeedFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.speedControlFunction = speedControlFunction;
    this.xLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    this.yLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    this.turningLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND);
    this.angleToPointTo = angleToPointTo;
    this.swerveDrivetrain = swerveSubsystem;
    this.redSide = redside;
    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double thetaFromCentre =
        Math.atan2(
            Vector3.subtract(new Vector3(centre), new Vector3(swerveDrivetrain.getState().Pose)).y,
            Vector3.subtract(new Vector3(centre), new Vector3(swerveDrivetrain.getState().Pose)).x);
    double tangentialSpeed = tangentSpdFunction.getAsDouble();
    tangentialSpeed =
        Math.abs(tangentialSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? tangentialSpeed : 0.0;

    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_PERCENT_SPEED_RANGE * (speedControlFunction.getAsDouble()))
            + Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT;

    double xSpeed = tangentialSpeed * Math.cos(thetaFromCentre + Math.PI / 2f);
    double ySpeed = tangentialSpeed * Math.sin(thetaFromCentre + Math.PI / 2f);

    // 2. Normalize inputs
    double length = xSpeed * xSpeed + ySpeed * ySpeed; // actually length squared
    if (length > 1d) {
      length = Math.sqrt(length);
      xSpeed /= length;
      ySpeed /= length;
    }

    // Apply Square (will be [0,1] since `speed` is [0,1])
    xSpeed = xSpeed * xSpeed * Math.signum(xSpeed);
    ySpeed = ySpeed * ySpeed * Math.signum(ySpeed);
    // 3. Apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? ySpeed : 0.0;

    // 4. Make the driving smoother
    // This is a double between TELE_DRIVE_SLOW_MODE_SPEED_PERCENT and
    // TELE_DRIVE_FAST_MODE_SPEED_PERCENT

    // Applies slew rate limiter
    xSpeed =
        xLimiter.calculate(xSpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed =
        yLimiter.calculate(ySpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    DogLog.log("Commands/joystickCommand/xSpeed", xSpeed);
    DogLog.log("Commands/joystickCommand/ySpeed", ySpeed);
    DogLog.log("Information/fieldCentric", fieldRelativeFunction.getAsBoolean());
    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    double turn =
        swerveDrivetrain.calculateRequiredRotationalRate(
            new Rotation2d(angleToPointTo.getAsDouble()));

    DogLog.log("Commands/joystickCommand/turnReq", turn);

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
