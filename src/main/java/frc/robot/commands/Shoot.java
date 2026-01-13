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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// it will need to rotate swerve and move arm

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem shooter;
  private final SwerveSubsystem drivetrain;
  private final ArmSubsystem arm;

  // private final SwerveSubsystem swerve;

  static final float MAX_TIME = 10f;
  static final float angularTolerance = 1f;
  static final float maxRotSpeed = .1f;

  float timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(ShooterSubsystem shooter, SwerveSubsystem drivetrain, ArmSubsystem arm) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, drivetrain, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.rampUp();
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
    arm.setTargetDegrees(Math.atan2(Math.sqrt(Math.pow(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, 2) + Math.pow(Vector3.subtract(new Vector3(drivetrain.getPose()), target).z, 2)), target.y));
    drivetrain.setRobotSpeeds(
      new ChassisSpeeds(
        drivetrain.getRobotSpeeds().vxMetersPerSecond,
        drivetrain.getRobotSpeeds().vyMetersPerSecond,
        -1 * MiscMath.clamp(Math.atan2(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, Vector3.subtract(new Vector3(drivetrain.getPose()), target).z) - drivetrain.getPose().getRotation().getRadians(), -maxRotSpeed, maxRotSpeed)
      )
    );
  }

  private boolean pointingAtTarget(Vector3 target) {
    boolean hullAimed = Math.atan2(Vector3.subtract(new Vector3(drivetrain.getPose()), target).x, Vector3.subtract(new Vector3(drivetrain.getPose()), target).z) - drivetrain.getPose().getRotation().getRadians() <= angularTolerance;
    boolean armAimed = arm.atTarget(angularTolerance);
    return hullAimed && armAimed;
  }

  protected Vector3 positionToTarget(Pose3d target) {
    BuiltInAccelerometer acc = new BuiltInAccelerometer();
    Vector3 a = Vector3.mult(new Vector3(acc.getX(), acc.getY(), acc.getZ()), -1);

    Vector3 v = Vector3.mult(new Vector3(drivetrain.getFieldSpeeds().vxMetersPerSecond, 0, drivetrain.getFieldSpeeds().vyMetersPerSecond), -1);

    Pose3d gunOffset = MiscMath.RotatedPosAroundVertical(Constants.Shooter.offset, drivetrain.getPose().getRotation().getRadians());
    Vector3 gunPos = Vector3.add(new Vector3(drivetrain.getPose()), new Vector3(gunOffset));
    Vector3 p = Vector3.subtract(new Vector3(target), gunPos);

    float s = Constants.Shooter.projectileInitSpeed;

    float[] coefficients = new float[5];
    coefficients[0] = (float) ((Math.pow(a.x, 2f) + Math.pow(a.y, 2f) + Math.pow(a.z, 2f)) / 4f);
    coefficients[1] = (a.x * v.x + a.y * v.y + a.z * v.z);
    coefficients[2] =
        (float)
            (Math.pow(v.x, 2f)
                + p.x * a.x
                + Math.pow(v.y, 2f)
                + p.y * a.y
                + Math.pow(v.z, 2f)
                + p.z * a.z
                - Math.pow(s, 2f));
    coefficients[3] = 2f * (p.x * v.x + p.y * v.y + p.z * v.z);
    coefficients[4] = (float) (Math.pow(p.x, 2f) + Math.pow(p.y, 2f) + Math.pow(p.z, 2f));

    float timeOfFlight = Polynomials.newtonRaphson(MAX_TIME, 5, 5f, coefficients);

    // if (timeOfFlight == -Mathf.Infinity) return targetedObj.transform.position;

    Vector3 accPlusGravity = Vector3.add(a, new Vector3(0, 9.8f, 0));
    return Vector3.add(
        p,
        Vector3.mult(v, timeOfFlight),
        Vector3.mult(accPlusGravity, (float) Math.pow(timeOfFlight, 2) * 1 / 2f));
  }
}
