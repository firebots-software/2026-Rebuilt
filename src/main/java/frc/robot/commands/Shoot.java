// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtils.MiscMath;
import frc.robot.MathUtils.Polynomials;
import frc.robot.MathUtils.Vector3;
import frc.robot.commands.SwerveCommands.SwerveJoystickCommand;
//import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// it will need to rotate swerve and move arm

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  //private final ShooterSubsystem shooter;
  private final SwerveSubsystem drivetrain;
  private final BooleanSupplier redside;

  static final float MAX_TIME = 10f;
  static final float angularTolerance = .1f;
  static final float maxRotSpeed = .1f;

  static final double shooterAngleDeg = 75;

  public static double targetAngle = 0;

  float timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(SwerveSubsystem drivetrain/* , ShooterSubsystem shooter*/, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    //this.shooter = shooter;
    this.redside = redside;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d target = redside.getAsBoolean() ? Constants.Landmarks.RED_HUB : Constants.Landmarks.BLUE_HUB;
    //shooter.rampUp(shootingSpeed(target, 5));
    //pointAtTarget(positionToTarget(target, 5));
    // if (shooter.atSpeed() && pointingAtTarget(positionToTarget(target, 5))) {
    //   shooter.runPreShooterAtRPS(10);
    // }
    targetAngle = targetAngle(target);

    DogLog.log("Shoot/isPointing", pointingAtTarget(positionToTarget(target, 5)));

    for (int i = 1; i <= 5; i++) {
      DogLog.log("Shoot/shootSpeed/" + i + "prec", shootingSpeed(target, i));
    }

    DogLog.log("Shoot/tof", 2 * shootingSpeed(target, 5) * Math.sin(Math.toRadians(shooterAngleDeg)) / 9.81);

    DogLog.log("Shoot/target", target);
    for (int i = 1; i <= 5; i++) {
      DogLog.log("Shoot/positionTargeting/" + i + "prec", new Pose3d(positionToTarget(target, i).x, positionToTarget(target, i).y, positionToTarget(target, i).z, new Rotation3d()));
    }

    DogLog.log("Shoot/ta", targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooter.stopAll();
    targetAngle = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double turnRate(Pose3d targetNoOffset) {
    return -1 * MiscMath.clamp(targetAngle(targetNoOffset) - drivetrain.getPose().getRotation().getRadians(), -maxRotSpeed, maxRotSpeed);
  }

  public double targetAngle(Pose3d targetNoOffset) {
    Vector3 target = positionToTarget(targetNoOffset, 5);
    return Math.atan2(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, Vector3.subtract(new Vector3(drivetrain.getPose()), target).y);
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
      correctedPos = Vector3.add(correctedPos, Vector3.mult(relativeVel, tof - prevTof));
      correctedSpeed = speedForDist(Vector3.subtract(correctedPos, gunPos).magnitude());
      prevTof = tof;
    }

    return correctedSpeed;
  }

  private double speedForDist(double d) {
    return Math.sqrt(d * 9.81 / Math.sin(Math.toRadians(shooterAngleDeg) * 2));
  }

  protected Vector3 positionToTarget(Pose3d target, int precision) { //fixed shooting speed, also Z is altitude btw
    Pose3d gunOffset = MiscMath.RotatedPosAroundVertical(Constants.Shooter.offset, drivetrain.getPose().getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getPose()), new Vector3(gunOffset));
    Vector3 relativePos = Vector3.subtract(new Vector3(target), gunPos);

    double timeOfFlight = 2 * shootingSpeed(target, precision) * Math.sin(Math.toRadians(shooterAngleDeg)) / 9.81;

    Vector3 relativeVel = Vector3.mult(new Vector3(drivetrain.getFieldSpeeds().vxMetersPerSecond, drivetrain.getFieldSpeeds().vyMetersPerSecond, 0), -1);

    return Vector3.add(
        new Vector3(target),
        Vector3.mult(relativeVel, timeOfFlight));
  }
}
