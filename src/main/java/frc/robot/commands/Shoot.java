// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem shooter;

  private final HopperSubsystem hopper;
  private final CommandSwerveDrivetrain drivetrain;
  private final BooleanSupplier redside;

  public static double targetAngle = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      HopperSubsystem hopper,
      BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.hopper = hopper;
    this.redside = redside;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d target =
        redside.getAsBoolean() ? Constants.Landmarks.RED_HUB : Constants.Landmarks.BLUE_HUB;
    shooter.setBallSpeed(
        Units.metersToFeet(
            shootingSpeed(target, Constants.Shooter.TARGETING_CALCULATION_PRECISION)));
    if (shooter.isAtSpeed() && pointingAtTarget()) {
      hopper.runHopper(Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND);
    } else {
      hopper.stop();
    }
    targetAngle = targetAngle(target);

    DogLog.log("Subsystems/ShooterSubsystem/Shoot/isPointing", pointingAtTarget());

    for (int i = 1; i <= Constants.Shooter.TARGETING_CALCULATION_PRECISION; i++) {
      DogLog.log(
          "Subsystems/ShooterSubsystem/Shoot/shootSpeedMetersPerSec/" + i + "prec",
          shootingSpeed(target, i));
    }

    DogLog.log(
        "Shoot/tof",
        2
            * shootingSpeed(target, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
            * Math.sin(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE_FROM_HORIZONTAL_DEGREES))
            / 9.81);

    DogLog.log("Subsystems/ShooterSubsystem/Shoot/target", target);
    for (int i = 1; i <= Constants.Shooter.TARGETING_CALCULATION_PRECISION; i++) {
      DogLog.log(
          "Subsystems/ShooterSubsystem/Shoot/positionTargeting/" + i + "prec",
          new Pose3d(
              positionToTarget(target, i).x,
              positionToTarget(target, i).y,
              positionToTarget(target, i).z,
              new Rotation3d()));
    }

    DogLog.log("Subsystems/ShooterSubsystem/Shoot/targetAngle", targetAngle);

    Pose3d shooterOffset =
        MiscMath.RotatedPosAroundVertical(
            Constants.Shooter.OFFSET_FROM_ROBOT_CENTER,
            drivetrain.getState().Pose.getRotation().getRadians());
    Vector3 shooterPos =
        Vector3.add(new Vector3(drivetrain.getState().Pose), new Vector3(shooterOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), shooterPos);
    DogLog.log("Subsystems/ShooterSubsystem/Shoot/distanceToTarget", relativePos.magnitude());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    hopper.stop();
    targetAngle = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
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
    double desiredRobotHullAngle = targetAngle;

    double robotHullAngle =
        (drivetrain.getState().Pose.getRotation().getRadians() + (2 * Math.PI)) % (2 * Math.PI);

    DogLog.log(
        "Subsystems/ShooterSubsystem/Shoot/rotationalError",
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
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond,
                0),
            -1);

    Pose3d shooterOffset =
        MiscMath.RotatedPosAroundVertical(
            Constants.Shooter.OFFSET_FROM_ROBOT_CENTER,
            drivetrain.getState().Pose.getRotation().getRadians());
    Vector3 shooterPos =
        Vector3.add(new Vector3(drivetrain.getState().Pose), new Vector3(shooterOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), shooterPos);

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
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, shooterPos).magnitude());
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
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond,
                0),
            -1);

    return Vector3.add(new Vector3(target), Vector3.mult(relativeVel, timeOfFlight));
  }
}
