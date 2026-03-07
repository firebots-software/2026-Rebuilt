package frc.robot.MathUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class Vector3 {
  public float x;
  public float y;
  public float z;

  public Vector3(float x, float y, float z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vector3(double x, double y, double z) {
    this.x = (float) x;
    this.y = (float) y;
    this.z = (float) z;
  }

  public Vector3(Pose2d pose) {
    x = (float) pose.getX();
    y = (float) pose.getY();
    z = 0;
  }

  public Vector3(Pose3d pose) {
    x = (float) pose.getX();
    y = (float) pose.getY();
    z = (float) pose.getZ();
  }

  public float magnitude() {
    return (float) Math.sqrt((Math.pow(x, 2f) + Math.pow(y, 2f) + Math.pow(z, 2f)));
  }

  public void plus(Vector3 other) {
    x += other.x;
    y += other.y;
    z += other.z;
  }

  public static Vector3 add(Vector3... a) {
    Vector3 newVec = new Vector3(0, 0, 0);
    for (Vector3 vec : a) {
      newVec.plus(vec);
    }
    return newVec;
  }

  public static Vector3 subtract(Vector3 a, Vector3 b) {
    return new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  public static Vector3 mult(Vector3 a, float m) {
    return new Vector3(a.x * m, a.y * m, a.z * m);
  }

  public static Vector3 mult(Vector3 a, double m) {
    return new Vector3(a.x * m, a.y * m, a.z * m);
  }
}
