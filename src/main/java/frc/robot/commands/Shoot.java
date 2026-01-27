// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Polynomials;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

// it will need to rotate swerve and move arm

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem shooter;
  private final CommandSwerveDrivetrain drivetrain;

  static final float MAX_TIME = 10f;
  static final float angularTolerance = 1f;
  static final float maxRotSpeed = .1f;

  static final double shooterAngleDeg = 75;

  float timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(ShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d target = new Pose3d();
    shooter.rampUp(shootingSpeed(target));
    pointAtTarget(positionToTarget(null));
    if (shooter.atSpeed() && pointingAtTarget(positionToTarget(null))) {
      shooter.runPreShooterAtRPS(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void pointAtTarget(Vector3 target) {
    drivetrain.applyFieldSpeeds(
      new ChassisSpeeds(
        drivetrain.getRobotSpeeds().vxMetersPerSecond,
        drivetrain.getRobotSpeeds().vyMetersPerSecond,
        -1 * MiscMath.clamp(Math.atan2(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, Vector3.subtract(new Vector3(drivetrain.getPose()), target).z) - drivetrain.getPose().getRotation().getRadians(), -maxRotSpeed, maxRotSpeed)
      )
    );
  }

  private boolean pointingAtTarget(Vector3 target) {
    boolean hullAimed = Math.atan2(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, Vector3.subtract(new Vector3(drivetrain.getPose()), target).z) - drivetrain.getPose().getRotation().getRadians() <= angularTolerance;
    return hullAimed;
  }

  protected double shootingSpeed(Pose3d target, int precision) {
    Vector3 relativeVel = Vector3.mult(new Vector3(drivetrain.getFieldSpeeds().vxMetersPerSecond, drivetrain.getFieldSpeeds().vyMetersPerSecond, 0), -1);

    Pose3d gunOffset = MiscMath.RotatedPosAroundVertical(Constants.Shooter.offset, drivetrain.getPose().getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getPose()), new Vector3(gunOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), gunPos);

    Vector3 correctedPos = new Vector3(target);
    double correctedSpeed = speedForDist(relativePos.magnitude());
    double prevTof = 0;
    for (int i = 0; i < precision; i++) {
      double tof = 2 * correctedSpeed * Math.sin(Math.toRadians(shooterAngleDeg)) / 9.81;
      correctedPos = Vector3.add(correctedPos, Vector3.mult(relativePos, tof - prevTof));
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, gunPos).magnitude());
      prevTof = tof;
    }

    return correctedSpeed;
  }

  private double speedForDist(double d) {
    return Math.sqrt(d * 9.81 / Math.sin(Math.toRadians(shooterAngleDeg) * 2));
  }

  protected Vector3 positionToTarget(Pose3d target) { //fixed shooting speed, also Z is altitude btw
    Pose3d gunOffset = MiscMath.RotatedPosAroundVertical(Constants.Shooter.offset, drivetrain.getPose().getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getPose()), new Vector3(gunOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), gunPos);

    double timeOfFlight = relativePos.magnitude() / (shootingSpeed(target) * Math.sin(Math.toRadians(shooterAngleDeg)));

    Vector3 relativeVel = Vector3.mult(new Vector3(drivetrain.getFieldSpeeds().vxMetersPerSecond, drivetrain.getFieldSpeeds().vyMetersPerSecond, 0), -1);

    return Vector3.add(
        new Vector3(target),
        Vector3.mult(relativeVel, timeOfFlight));
  }
}
