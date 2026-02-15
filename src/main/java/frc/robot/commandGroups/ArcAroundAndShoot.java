package frc.robot.commandGroups;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Vector3;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommandInArc;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcAroundAndShoot extends ParallelCommandGroup {

  private final CommandSwerveDrivetrain drivetrain;
  private final Pose3d targetNoOffset;

  public ArcAroundAndShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVelocitySupplier,
      Pose3d target,
      BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.targetNoOffset = target;

    addCommands(
        new SwerveJoystickCommandInArc(
            target,
            tangentialVelocitySupplier,
            (DoubleSupplier) (() -> 1f),
            (BooleanSupplier) (() -> true),
            (DoubleSupplier) (() -> targetAngle(target)),
            drivetrain,
            redside),
            
        new Shoot(shootingSpeed(target, Constants.Shooter.TARGETING_CALCULATION_PRECISION), () -> pointingAtTarget(), shooter, intake, hopper)
        
    );
  }

  public ArcAroundAndShoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      HopperSubsystem hopper,
      DoubleSupplier tangentialVelocitySupplier,
      BooleanSupplier redside) {
        this(
            drivetrain,
            shooter,
            intake,
            hopper,
            tangentialVelocitySupplier,
            redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB,
            redside
        );
    }

  public double targetAngle(Pose3d targetNoOffset) {
    Vector3 target =
        positionToTarget(targetNoOffset, Constants.Shooter.TARGETING_CALCULATION_PRECISION);
    return Math.atan2(
            Vector3.subtract(target, new Vector3(drivetrain.getState().Pose)).y,
            Vector3.subtract(target, new Vector3(drivetrain.getState().Pose)).x)
        + (Constants.Shooter.SHOOTS_BACKWARDS ? Math.PI : 0);
  }

  private boolean pointingAtTarget() {
    double desiredRobotHullAngle = targetAngle(targetNoOffset);

    double robotHullAngle =
        (drivetrain.getState().Pose.getRotation().getRadians() + (2 * Math.PI)) % (2 * Math.PI);

    DogLog.log(
        "Subsystems/ShooterSubsystem/Shoot/rotationalErrorRadians",
        Math.abs(desiredRobotHullAngle - robotHullAngle));
    boolean hullAimed =
        Math.abs(desiredRobotHullAngle - robotHullAngle)
            <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
    return hullAimed;
  }

  public boolean pointingAtTarget(CommandSwerveDrivetrain drivetrain) {
    double desiredRobotHullAngle = targetAngle(targetNoOffset);

    double robotHullAngle =
        (drivetrain.getState().Pose.getRotation().getRadians() + (2 * Math.PI)) % (2 * Math.PI);

    DogLog.log(
        "Subsystems/ShooterSubsystem/Shoot/rotationalErrorRadians",
        Math.abs(desiredRobotHullAngle - robotHullAngle));
    boolean hullAimed =
        Math.abs(desiredRobotHullAngle - robotHullAngle)
            <= Constants.Shooter.ANGULAR_TOLERANCE_FOR_AUTO_AIM_RAD;
    return hullAimed;
  }

  private double shootingSpeed(Pose3d target, int precision) { // meters per sec
    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vxMetersPerSecond,
                drivetrain.getFieldSpeeds().vyMetersPerSecond,
                0),
            -1);

    Pose3d gunOffset =
        MiscMath.RotatedPosAroundVertical(
            Constants.Shooter.OFFSET_FROM_ROBOT_CENTER,
            drivetrain.getState().Pose.getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getState().Pose), new Vector3(gunOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), gunPos);

    Vector3 correctedPos = new Vector3(target);
    double correctedSpeed = speedForDist(relativePos.magnitude());
    double prevTof = 0;
    for (int i = 0; i < precision; i++) {
      double tof =
          2
              * correctedSpeed
              * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES))
              / 9.81;
      correctedPos = Vector3.add(correctedPos, Vector3.mult(relativeVel, tof - prevTof));
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, gunPos).magnitude());
      prevTof = tof;
    }

    return correctedSpeed;
  }

  private double speedForDist(double d) {
    return Math.sqrt(
        d
            * 9.81
            / Math.sin(
                Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES) * 2));
  }

  private Vector3 positionToTarget(Pose3d target, int precision) {
    double timeOfFlight =
        2
            * shootingSpeed(target, precision)
            * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES))
            / 9.81;

    Vector3 relativeVel =
        Vector3.mult(
            new Vector3(
                drivetrain.getFieldSpeeds().vxMetersPerSecond,
                drivetrain.getFieldSpeeds().vyMetersPerSecond,
                0),
            -1);

    return Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));
  }
}
