package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunedLinearPath {
  public static class State {
    public Pose2d pose;
    public ChassisSpeeds speeds;

    public State() {
      this(Pose2d.kZero, new ChassisSpeeds());
    }

    public State(Pose2d pose, ChassisSpeeds speeds) {
      this.pose = pose;
      this.speeds = speeds;
    }
  }

  private TrapezoidProfile linearProfile;
  private TrapezoidProfile angularProfile;

  private Pose2d initialPose = Pose2d.kZero;
  private Rotation2d heading = Rotation2d.kZero;

  private TrapezoidProfile.State linearGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State linearStart = new TrapezoidProfile.State();

  private TrapezoidProfile.State angularGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State angularStart = new TrapezoidProfile.State();

  public TunedLinearPath(
      TrapezoidProfile.Constraints linear, TrapezoidProfile.Constraints angular) {
    this.linearProfile = new TrapezoidProfile(linear);
    this.angularProfile = new TrapezoidProfile(angular);
  }

  private static double calculateVelocityAtHeading(ChassisSpeeds speeds, Rotation2d heading) {
    // vel = <vx, vy> ⋅ <cos(heading), sin(heading)>
    // vel = vx * cos(heading) + vy * sin(heading)
    return speeds.vxMetersPerSecond * heading.getCos()
        + speeds.vyMetersPerSecond * heading.getSin();
  }

  private void setState(State current, Pose2d goal) {
    this.initialPose = current.pose;

    // pull out the translation from our initial pose to the target
    Translation2d translation = goal.getTranslation().minus(this.initialPose.getTranslation());
    // pull out distance and heading to the target
    double distance = translation.getNorm();
    this.heading = distance > 1e-6 ? translation.getAngle() : Rotation2d.kZero;
    this.linearGoal = new TrapezoidProfile.State(distance, 2);

    // start at current velocity in the direction of travel
    double vel = calculateVelocityAtHeading(current.speeds, this.heading);
    this.linearStart = new TrapezoidProfile.State(0, vel);
    this.angularStart =
        new TrapezoidProfile.State(
            current.pose.getRotation().getRadians(), current.speeds.omegaRadiansPerSecond);
    // wrap the angular goal so we take the shortest path
    this.angularGoal =
        new TrapezoidProfile.State(
            current.pose.getRotation().getRadians()
                + goal.getRotation().minus(current.pose.getRotation()).getRadians(),
            0);
  }

  private State calculate(double t) {
    // calculate our new distance and velocity in the desired direction of travel
    TrapezoidProfile.State linearState =
        this.linearProfile.calculate(t, this.linearStart, this.linearGoal);
    // calculate our new heading and rotational rate
    TrapezoidProfile.State angularState =
        this.angularProfile.calculate(t, this.angularStart, this.angularGoal);

    // x is m_state * cos(heading), y is m_state * sin(heading)
    Pose2d pose =
        new Pose2d(
            this.initialPose.getX() + linearState.position * this.heading.getCos(),
            this.initialPose.getY() + linearState.position * this.heading.getSin(),
            Rotation2d.fromRadians(angularState.position));
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearState.velocity * this.heading.getCos(),
            linearState.velocity * this.heading.getSin(),
            angularState.velocity);
    return new State(pose, speeds);
  }

  public final State calculate(double t, State current, Pose2d goal) {
    setState(current, goal);
    return calculate(t);
  }

  public final double totalTime() {
    return Math.max(this.linearProfile.totalTime(), this.angularProfile.totalTime());
  }

  public final boolean isFinished(double t) {
    return t >= totalTime();
  }
}
