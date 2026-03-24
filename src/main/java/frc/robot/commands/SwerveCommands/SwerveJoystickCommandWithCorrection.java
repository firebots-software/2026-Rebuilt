package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtils.Vector2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.VisionUtils.IntakeVisionTarget;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCommandWithCorrection extends Command {
  protected final DoubleSupplier xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedControlFunction;
  protected final BooleanSupplier fieldRelativeFunction, doPointing, redsideIfPointing;

  protected final CommandSwerveDrivetrain swerveDrivetrain;
  protected final BooleanSupplier doDriveAssist;

  private final IntakeVisionTarget targetResult;

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private boolean squaredTurn;

  public SwerveJoystickCommandWithCorrection(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      BooleanSupplier doPointing,
      BooleanSupplier redSideIfPointing,
      CommandSwerveDrivetrain swerveSubsystem,
      IntakeVisionTarget targetResult,
      BooleanSupplier doDriveAssist) {
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedControlFunction = speedControlFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.squaredTurn = true;
    this.swerveDrivetrain = swerveSubsystem;
    this.doPointing = doPointing;
    this.redsideIfPointing = redSideIfPointing;
    this.targetResult = targetResult;
    this.doDriveAssist = doDriveAssist;

    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  // Sets everything, not field relative
  public SwerveJoystickCommandWithCorrection(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      CommandSwerveDrivetrain swerveSubsystem,
      IntakeVisionTarget targetResult) {

    this(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        () -> false,
        () -> false,
        () -> false,
        swerveSubsystem,
        targetResult,
        () -> false);
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
    double length = xSpeed * xSpeed + ySpeed * ySpeed; // acutally length squared
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
    xSpeed = xSpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed = ySpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    turningSpeed =
        turningSpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;

    // Final values to apply to drivetrain
    final double x = xSpeed;
    final double y = ySpeed;

    Pose2d curPose = swerveDrivetrain.getCurrentState().Pose;

    Pose2d targetPose = targetResult.pose();

    // double distToTarget = Constants.IntakeVision.CAM_HEIGHT_METERS /
    // Math.tan(Units.degreesToRadians(intakeVision.getPitch()));
    // targetPose =
    //     new Pose2d(
    //         curPose.getX()
    //             + distToTarget
    //                 * Math.cos(
    //                     Units.degreesToRadians(intakeVision.getYaw())
    //                         + curPose.getRotation().getRadians()),
    //         curPose.getY()
    //             + distToTarget
    //                 * Math.sin(
    //                     Units.degreesToRadians(intakeVision.getYaw())
    //                         + curPose.getRotation().getRadians()),
    //         new Rotation2d());

    double turn =
        (doPointing.getAsBoolean())
            ? (swerveDrivetrain.calculateRequiredRotationalRate(
                swerveDrivetrain.travelAngleTo(
                    ((redsideIfPointing.getAsBoolean())
                        ? (Constants.Landmarks.RED_HUB_2D)
                        : (Constants.Landmarks.BLUE_HUB_2D)))))
            : (turningSpeed);

    if (Math.abs(turningSpdFunction.getAsDouble()) > Constants.IntakeVision.OVERRIDE_ROT_INPUT
        && doDriveAssist.getAsBoolean()
        && !doPointing.getAsBoolean()) {
      turn +=
          swerveDrivetrain.calculateRequiredRotationalRate(
              new Rotation2d(
                  Math.atan2(targetPose.getY() - curPose.getY(), targetPose.getX() - curPose.getX())
                      + curPose.getRotation().getRadians()));
    }

    double velocityX = x;
    double velocityY = y;
    if (doDriveAssist.getAsBoolean()) {
      velocityX =
          MathUtil.clamp(
              x + translationAssist(targetPose).x,
              -Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
              Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
      velocityY =
          MathUtil.clamp(
              y + translationAssist(targetPose).y,
              -Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND,
              Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
    }

    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    SwerveRequest drive =
        fieldRelativeFunction.getAsBoolean()
            ? fieldCentricDrive
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(turn)
            : robotCentricDrive
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(turn);

    // Applies request
    this.swerveDrivetrain.setControl(drive);
  } // Drive counterclockwise with negative X (left))

  public Vector2 translationAssist(Pose2d targetPose) {
    int n = 2;

    double p1x = swerveDrivetrain.getCurrentState().Pose.getX();
    double p1y = swerveDrivetrain.getCurrentState().Pose.getY();

    Pose2d p2 =
        new Pose2d(
            p1x + Constants.IntakeVision.lookAheadTime * xSpdFunction.getAsDouble(),
            p1y + Constants.IntakeVision.lookAheadTime * ySpdFunction.getAsDouble(),
            new Rotation2d());

    double dist =
        Math.abs(
                ((p2.getY() - p1y) * targetPose.getX())
                    - ((p2.getX() - p1x) * targetPose.getY())
                    + p2.getX() * p1y
                    - p2.getY() * p1x)
            / Math.sqrt(Math.pow(((p2.getY() - p1y)), 2) + Math.pow((p2.getX() - p1x), 2));

    double assistMagnitude = Math.pow((dist * Constants.IntakeVision.kP), 1.0 / n);
    double assistDirection = Math.atan2(p1y - targetPose.getY(), p1x - targetPose.getX());

    return new Vector2(
        assistMagnitude * Math.cos(assistDirection), assistMagnitude * Math.sin(assistDirection));
  }

  @Override
  public void end(boolean interrupted) {
    // Applies SwerveDriveBrake (brakes the robot by turning wheels)
    this.swerveDrivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
