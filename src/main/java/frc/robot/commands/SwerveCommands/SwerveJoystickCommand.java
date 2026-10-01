package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Targeting;
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
      capper,
      braking;

  protected final CommandSwerveDrivetrain swerveDrivetrain;

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
      BooleanSupplier capper,
      BooleanSupplier braking) {
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
    this.braking = braking;

    addRequirements(swerveDrivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // x+ front, x- back; y+ left, y- right; turn+ ccw, turn- cw
    double xSpeed = xSpdFunction.getAsDouble(); // xSpeed is actually front back (front +, back -)
    double ySpeed = ySpdFunction.getAsDouble(); // ySpeed is actually left right (left +, right -)
    double turningSpeed = turningSpdFunction.getAsDouble(); 

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

    if (Math.abs(turningSpeed) < Constants.OI.RIGHT_JOYSTICK_DEADBAND) {
        turningSpeed = 0.0;
    } else if (squaredTurn) {
        turningSpeed = Math.abs(turningSpeed * turningSpeed * turningSpeed);
    }
   
    // slew rate
    xSpeed = xSpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    ySpeed = ySpeed * Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    turningSpeed = turningSpeed * Constants.Swerve.PHYSICAL_MAX_ANGLUAR_SPEED_RADIANS_PER_SECOND;

    double speedMagnitude = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    if (capper.getAsBoolean()) {
      xSpeed *= 0.3;
      ySpeed *= 0.3;
      // if (speedMagnitude > 2.0) {
      //   xSpeed *= (2.0 / speedMagnitude);
      //   ySpeed *= (2.0 / speedMagnitude);
      // }
    }

    // Final values to apply to drivetrain
    final double x = xSpeed;
    final double y = ySpeed;

    double turn =
        (doPointing.getAsBoolean())
            ? (swerveDrivetrain.calculateRequiredRotationalRateWithFF(
                swerveDrivetrain.getVirtualTarget(redsideIfPointing, () -> false)))
            : (turningSpeed);

    if (doPassing.getAsBoolean()) {
      turn =
          swerveDrivetrain.calculateRequiredRotationalRateWithFF(
              swerveDrivetrain.getPassingTarget(redsideIfPointing));
    }

    // 5. Applying the drive request on the swerve drivetrain
    // Uses SwerveRequestFieldCentric (from java.frc.robot.util to apply module optimization)
    SwerveRequest drive =
        fieldRelativeFunction.getAsBoolean()
            ? fieldCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn)
            : robotCentricDrive.withVelocityX(x).withVelocityY(y).withRotationalRate(turn);

    boolean isPointed =
        Targeting.pointingAtTarget(
            swerveDrivetrain
                    .travelAngleTo(
                        new Pose2d(
                            swerveDrivetrain.getVirtualTarget(redsideIfPointing, () -> false),
                            new Rotation2d()))
                    .getRadians()
                + Math.PI,
            swerveDrivetrain);
    boolean intentionallyStationary =
        (xSpdFunction.getAsDouble() < 0.1)
            && (ySpdFunction.getAsDouble() < 0.1)
            && (turningSpdFunction.getAsDouble() < 0.1);
    if (braking.getAsBoolean()
        && doPointing.getAsBoolean()
        && isPointed
        && intentionallyStationary) {
      swerveDrivetrain.brakeSwerve();
      DogLog.log("IsItBraking?", true);
    } else {
      DogLog.log("IsItBraking?", false);
      this.swerveDrivetrain.setControl(drive);
    }
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
