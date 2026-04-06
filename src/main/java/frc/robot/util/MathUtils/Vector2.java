package frc.robot.util.MathUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Vector2 {
  public float x;
  public float y;

  public Vector2(float x, float y) {
    this.x = x;
    this.y = y;
  }

  public Vector2(double x, double y) {
    this.x = (float) x;
    this.y = (float) y;
  }

  public Vector2(Pose2d pose) {
    x = (float) pose.getX();
    y = (float) pose.getY();
  }

  public float magnitude() {
    return (float) Math.sqrt((Math.pow(x, 2f) + Math.pow(y, 2f)));
  }

  public void plus(Vector2 other) {
    x += other.x;
    y += other.y;
  }

  public static Vector2 add(Vector2... a) {
    Vector2 newVec = new Vector2(0, 0);
    for (Vector2 vec : a) {
      newVec.plus(vec);
    }
    return newVec;
  }

  public static Vector2 subtract(Vector2 a, Vector2 b) {
    return new Vector2(a.x - b.x, a.y - b.y);
  }

  public static Vector2 mult(Vector2 a, float m) {
    return new Vector2(a.x * m, a.y * m);
  }

  public static Vector2 mult(Vector2 a, double m) {
    return new Vector2(a.x * m, a.y * m);
  }

  public static Pose2d toPose2d(Vector2 a) {
    return new Pose2d(a.x, a.y, new Rotation2d());
  }

  public static Vector2 fromPose2d(Pose2d a) {
    return new Vector2(a.getX(), a.getY());
  }

  public static double dist(Vector2 a, Vector2 b) {
    return Vector2.subtract(a, b).magnitude();
  }
}