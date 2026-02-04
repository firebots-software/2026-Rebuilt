package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCommand extends Command {
  protected final DoubleSupplier xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedControlFunction;

  protected final BooleanSupplier fieldRelativeFunction;

  // Limits rate of change (in this case x, y, and turning movement)
  protected final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  protected final CommandSwerveDrivetrain swerveDrivetrain;
  protected BooleanSupplier fixedRotation;
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private boolean squaredTurn;
  private BooleanSupplier redSide, leftL1, rightL1;

  public SwerveJoystickCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      CommandSwerveDrivetrain swerveSubsystem) {
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedControlFunction = speedControlFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.squaredTurn = true;
    this.xLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    this.yLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND);
    this.turningLimiter =
        new SlewRateLimiter(
            Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND);
    this.swerveDrivetrain = swerveSubsystem;
    this.fixedRotation = () -> false;
    this.redSide = () -> false;
    this.leftL1 = () -> false;
    this.rightL1 = () -> false;
    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  // tim's
  public SwerveJoystickCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      BooleanSupplier redSide,
      BooleanSupplier targetRotationSupplier,
      CommandSwerveDrivetrain swerveSubsystem) {
    this(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        fieldRelativeFunction,
        swerveSubsystem);
    this.fixedRotation = targetRotationSupplier;
    this.redSide = redSide;
    this.leftL1 = () -> false;
    this.rightL1 = () -> false;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.getAsDouble(); // xSpeed is actually front back (front +, back -)
    double ySpeed = ySpdFunction.getAsDouble(); // ySpeed is actually left right (left +, right -)
    double turningSpeed =
        turningSpdFunction.getAsDouble(); // turning speed is (anti-clockwise +, clockwise -)

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
    if (squaredTurn) {
      turningSpeed = turningSpeed * turningSpeed * Math.signum(turningSpeed);
    }
    // 3. Apply deadband
    xSpeed = Math.abs(xSpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OI.LEFT_JOYSTICK_DEADBAND ? ySpeed : 0.0;
    turningSpeed =
        Math.abs(turningSpeed) > Constants.OI.RIGHT_JOYSTICK_DEADBAND ? turningSpeed : 0.0;

    // 4. Make the driving smoother
    // This is a double between TELE_DRIVE_SLOW_MODE_SPEED_PERCENT and
    // TELE_DRIVE_FAST_MODE_SPEED_PERCENT
    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_PERCENT_SPEED_RANGE * (speedControlFunction.getAsDouble()))
            + Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT;

    // Applies slew rate limiter
    xSpeed =
        xLimiter.calculate(xSpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed =
        yLimiter.calculate(ySpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    turningSpeed =
        turningLimiter.calculate(turningSpeed)
            * driveSpeed
            * Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;

    // Final values to apply to drivetrain
    double turn = turningSpeed;

    DogLog.log("Commands/joystickCommand/xSpeed", xSpeed);
    DogLog.log("Commands/joystickCommand/ySpeed", ySpeed);
    DogLog.log("Commands/joystickCommand/turningSpeed", turningSpeed);
    DogLog.log("Information/fieldCentric", fieldRelativeFunction.getAsBoolean());
    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    if (fixedRotation.getAsBoolean()) {
      turn = swerveDrivetrain.calculateRequiredRotationalRate(new Rotation2d(Shoot.targetAngle));
    }

    DogLog.log("Commands/joystickCommand/turnReq", turn);
    DogLog.log("Commands/joystickCommand/usingShootCmdRot", fixedRotation.getAsBoolean());

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
