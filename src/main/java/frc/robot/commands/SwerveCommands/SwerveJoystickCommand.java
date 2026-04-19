package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCommand extends Command {
  protected final DoubleSupplier xSpdFunction,
      ySpdFunction,
      turningSpdFunction,
      speedControlFunction;
  protected final BooleanSupplier fieldRelativeFunction,
      doPointing,
      doPassing,
      redsideIfPointing,
      capper;

  protected final CommandSwerveDrivetrain swerveDrivetrain;

  // Limits rate of change (in this case x, y, and turning movement)
  protected final SlewRateLimiter xLimiter, yLimiter;

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private boolean squaredTurn;

  public SwerveJoystickCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      BooleanSupplier doPointing,
      BooleanSupplier doPassing,
      BooleanSupplier redSideIfPointing,
      CommandSwerveDrivetrain swerveSubsystem,
      BooleanSupplier capper) {
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.speedControlFunction = speedControlFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.squaredTurn = true;
    this.swerveDrivetrain = swerveSubsystem;
    this.doPointing = doPointing;
    this.doPassing = doPassing;
    this.redsideIfPointing = redSideIfPointing;
    this.capper = capper;
    this.xLimiter = new SlewRateLimiter(3.5);
    this.yLimiter = new SlewRateLimiter(3.5);

    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  // Sets everything, not field relative
  public SwerveJoystickCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      CommandSwerveDrivetrain swerveSubsystem,
      BooleanSupplier capper) {

    this(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        () -> false,
        () -> false,
        () -> false,
        () -> false,
        swerveSubsystem,
        capper);
  }

  public SwerveJoystickCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier turningSpdFunction,
      DoubleSupplier speedControlFunction,
      CommandSwerveDrivetrain swerveSubsystem,
      boolean squaredTurn,
      BooleanSupplier capper) {

    this(
        frontBackFunction,
        leftRightFunction,
        turningSpdFunction,
        speedControlFunction,
        swerveSubsystem,
        capper);
    this.squaredTurn = squaredTurn;
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

    // Applies slew rate limiter
    xSpeed = xSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed = ySpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    turningSpeed = turningSpeed * Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;

    double speedMagnitude = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    if (capper.getAsBoolean()) {
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
      if (speedMagnitude > 2.5) {
        xSpeed *= (2.5 / speedMagnitude);
        ySpeed *= (2.5 / speedMagnitude);
      }
    }

    // Final values to apply to drivetrain
    final double x = xSpeed;
    final double y = ySpeed;

    // final double turn =
    //     (doPointing.getAsBoolean())
    //         ? (swerveDrivetrain.calculateRequiredRotationalRate(
    //             swerveDrivetrain.travelAngleTo(
    //                 ((redsideIfPointing.getAsBoolean())
    //                     ? (Constants.Landmarks.RED_HUB_2D)
    //                     : (Constants.Landmarks.BLUE_HUB_2D)))))
    //         : (turningSpeed);

    double turn =
        (doPointing.getAsBoolean())
            ? (swerveDrivetrain.calculateRequiredRotationalRateWithFF(
                swerveDrivetrain.getVirtualTarget(redsideIfPointing, () -> false)))
            : (turningSpeed);

    if (doPassing.getAsBoolean())
      turn =
          swerveDrivetrain.calculateRequiredRotationalRateWithFF(
              swerveDrivetrain.getPassingTarget(redsideIfPointing));

    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    SwerveRequest drive =
        fieldRelativeFunction.getAsBoolean()
            ? fieldCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn)
            : robotCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn);

    // Applies request
    this.swerveDrivetrain.setControl(drive);
  } // Drive counterclockwise with negative X (left))

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
