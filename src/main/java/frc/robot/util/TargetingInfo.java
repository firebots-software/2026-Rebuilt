package frc.robot.util;

import frc.robot.util.MathUtils.Vector3;

public class TargetingInfo {
  private double speed;
  private double tof;
  private Vector3 pos;

  public TargetingInfo(double speed, double tof, Vector3 pos) {
    this.speed = speed;
    this.tof = tof;
    this.pos = pos;
  }

  public double getSpeed() {
    return speed;
  }

  public double getToF() {
    return tof;
  }

  public Vector3 getPosition() {
    return pos;
  }
}
