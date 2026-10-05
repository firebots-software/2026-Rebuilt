package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Translation2d;

public class SwerveJoystickCommandWithPointing extends Command {
  protected final DoubleSupplier xSpdFunction, ySpdFunction;

  protected Supplier<Translation2d> poseToTarget;

  protected final DoubleSupplier speedControlFunction;

  protected final BooleanSupplier fieldRelativeFunction;

  protected final CommandSwerveDrivetrain swerveDrivetrain;
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  public SwerveJoystickCommandWithPointing(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier speedControlFunction,
      BooleanSupplier fieldRelativeFunction,
      Supplier<Translation2d> poseToTarget,
      CommandSwerveDrivetrain swerveSubsystem) {
    this.xSpdFunction = frontBackFunction;
    this.ySpdFunction = leftRightFunction;
    this.speedControlFunction = speedControlFunction;
    this.fieldRelativeFunction = fieldRelativeFunction;
    this.swerveDrivetrain = swerveSubsystem;
    this.poseToTarget = poseToTarget;
    // Adds the subsystem as a requirement (prevents two commands from acting on subsystem at once)
    addRequirements(swerveDrivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // x+ front, x- back; y+ left, y- right; turn+ ccw, turn- cw
    double xSpeed = xSpdFunction.getAsDouble(); // xSpeed is actually front back (front +, back -)
    double ySpeed = ySpdFunction.getAsDouble(); // ySpeed is actually left right (left +, right -)

    double magnitude = Math.hypot(xSpeed, ySpeed);

    if (magnitude < Constants.OI.LEFT_JOYSTICK_DEADBAND) {
      xSpeed = 0.0;
      ySpeed = 0.0;
    } else {
      double xDir = xSpeed / magnitude;
      double yDir = ySpeed / magnitude;

      magnitude =
          (magnitude - Constants.OI.LEFT_JOYSTICK_DEADBAND)
              / (1.0 - Constants.OI.LEFT_JOYSTICK_DEADBAND);

      double magnitudeSquared = magnitude * magnitude;
      magnitudeSquared *= Constants.Swerve.GLOBAL_SWERVE_MULT;

      xSpeed = xDir * magnitudeSquared;
      ySpeed = yDir * magnitudeSquared;
    }

    double driveSpeed =
        (Constants.Swerve.TELE_DRIVE_PERCENT_SPEED_RANGE * (speedControlFunction.getAsDouble()))
            + Constants.Swerve.TELE_DRIVE_SLOW_MODE_SPEED_PERCENT;

    xSpeed = xSpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed = ySpeed * driveSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;

    // Final values to apply to drivetrain

    DogLog.log("Commands/joystickCommand/xSpeed", xSpeed);
    DogLog.log("Commands/joystickCommand/ySpeed", ySpeed);
    DogLog.log("Information/fieldCentric", fieldRelativeFunction.getAsBoolean());
    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    double turn = swerveDrivetrain.calculateRequiredRotationalRateWithFF(poseToTarget.get());

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
