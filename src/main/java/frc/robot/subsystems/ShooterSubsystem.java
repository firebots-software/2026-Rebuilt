// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.NetworkChannel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.MathUtils.Polynomials;
import frc.robot.MathUtils.Vector3;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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

    float timeOfFlight = Polynomials.newtonRaphson(s == 0 ? 0f : p.magnitude() / s, 5, 5f, coefficients);

    //if (timeOfFlight == -Mathf.Infinity) return targetedObj.transform.position;

    return Vector3.add(
            p, 
            Vector3.mult(v, timeOfFlight), 
            Vector3.mult(Vector3.add(a, new Vector3(0, 9.8f, 0)), 
                        (float) Math.pow(timeOfFlight, 2) * 1/2f));
  }
}
