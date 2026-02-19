package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/** This Command drives the robot in a linear path to a specific pose. */
public class DriveToPose extends Command {
  private final CommandSwerveDrivetrain swerve;

  // Initialize the LinearPath and the LinearPath.State to a null value
  private LinearPath path = null;
  private LinearPath.State pathState = null;

  // Initialize the Target Pose and the Target Pose Supplier to a null value
  private Pose2d targetPose = null;
  private Supplier<Pose2d> targetPoseSupplier = null;

  private final PIDController xController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kPX,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kIX,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kDX);
  private final PIDController yController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kPY,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kIY,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kDY);
  private final PIDController headingController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kPR,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kIR,
          Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kDR);

  double startTime;

  /**
   * @param swerve Swerve Subsystem.
   * @param targetPoseSupplier Target Pose Supplier (for changing values of pose not just runtime)
   */
  public DriveToPose(CommandSwerveDrivetrain swerve, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.targetPose = targetPoseSupplier.get();

    path =
        new LinearPath(
            new TrapezoidProfile.Constraints(
                Constants.Swerve.WHICH_SWERVE_ROBOT
                    .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                    .maxVelocityLinear,
                Constants.Swerve.WHICH_SWERVE_ROBOT
                    .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                    .maxAccelerationLinear),
            new TrapezoidProfile.Constraints(
                Constants.Swerve.WHICH_SWERVE_ROBOT
                    .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                    .maxVelocityAngular,
                Constants.Swerve.WHICH_SWERVE_ROBOT
                    .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                    .maxAccelerationAngular)); // constants

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    startTime = Utils.getCurrentTimeSeconds();

    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }

    pathState =
        new LinearPath.State(swerve.getCurrentState().Pose, swerve.getCurrentState().Speeds);

    DogLog.log("Swerve/Drive To Pose/Init Target Pose X", targetPose.getX());
    DogLog.log("Swerve/Drive To Pose/Init Target Pose Y", targetPose.getY());
    DogLog.log(
        "Swerve/Drive To Pose/Init Target Pose Rotation", targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Utils.getCurrentTimeSeconds() - startTime;

    if (pathState == null) return;

    pathState = path.calculate(currTime, pathState, targetPose);

    // Generate the next speeds for the robot
    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            pathState.speeds.vxMetersPerSecond
                + xController.calculate(
                    swerve.getCurrentState().Pose.getX(), pathState.pose.getX()),
            pathState.speeds.vyMetersPerSecond
                + yController.calculate(
                    swerve.getCurrentState().Pose.getY(), pathState.pose.getY()),
            pathState.speeds.omegaRadiansPerSecond
                + headingController.calculate(
                    swerve.getCurrentState().Pose.getRotation().getRadians(),
                    pathState.pose.getRotation().getRadians()));

    swerve.applyOneFieldSpeeds(speeds);
  }

  private boolean atPosition() {
    return (Math.abs(swerve.getCurrentState().Pose.getX() - targetPose.getX())
            <= Constants.Swerve.targetPositionError)
        && (Math.abs(swerve.getCurrentState().Pose.getY() - targetPose.getY())
            <= Constants.Swerve.targetPositionError)
        && (Math.abs(
                swerve.getCurrentState().Pose.getRotation().getRadians()
                    - targetPose.getRotation().getRadians())
            <= Constants.Swerve.targetAngleError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition();
  }
}
