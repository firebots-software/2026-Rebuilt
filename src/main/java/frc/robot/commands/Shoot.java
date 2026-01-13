// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MathUtils.Polynomials;
import frc.robot.MathUtils.Vector3;
import frc.robot.subsystems.ShooterSubsystem;

// it will need to rotate swerve and move arm

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem shooter;
  //private final SwerveSubsystem swerve;

  static final float MAX_TIME = 100f;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(ShooterSubsystem shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.rampUp();
    if (shooter.atSpeed()) {
      shooter.runPreShooterAtRPS(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    shooter.runPreShooterAtRPS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooter.objDetected();
  }

  protected Vector3 positionToTarget() {
    Vector3 a = new Vector3(0,0,0);
    Vector3 v = new Vector3(0,0,0);
    Vector3 p = new Vector3(0,0,0);
    float s = 0;

    float[] coefficients = new float[5];
    coefficients[0] = (float) ((Math.pow(a.x, 2f) + Math.pow(a.y, 2f) + Math.pow(a.z, 2f)) / 4f);
    coefficients[1] = (a.x * v.x + a.y * v.y + a.z * v.z);
    coefficients[2] = (float) (Math.pow(v.x, 2f) + p.x * a.x + Math.pow(v.y, 2f) + p.y * a.y + Math.pow(v.z, 2f) + p.z * a.z - Math.pow(s, 2f));
    coefficients[3] = 2f * (p.x * v.x + p.y * v.y + p.z * v.z);
    coefficients[4] = (float) (Math.pow(p.x, 2f) + Math.pow(p.y, 2f) + Math.pow(p.z, 2f));

    float timeOfFlight = Polynomials.newtonRaphson(MAX_TIME, 5, 5f, coefficients);

    //if (timeOfFlight == -Mathf.Infinity) return targetedObj.transform.position;

    return Vector3.add(
            p, 
            Vector3.mult(v, timeOfFlight), 
            Vector3.mult(Vector3.add(a, new Vector3(0, 9.8f, 0)), 
                        (float) Math.pow(timeOfFlight, 2) * 1/2f));
  }
}
