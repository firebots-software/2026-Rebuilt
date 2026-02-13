package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private final WheelForceCalculator wheelForceCalculator;
  private WheelForceCalculator.Feedforwards feedforwards;

  private double previousTime;
  private ChassisSpeeds prev = new ChassisSpeeds();

  /**
   * @param swerve Swerve Subsystem.
   * @param targetPose Target Pose (static).
   */
  //   public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose) { //dont use this
  //     // Use addRequirements() here to declare subsystem dependencies.
  //     this.swerve = swerve;
  //     this.targetPose = targetPose;

  //     path =
  //         new LinearPath(
  //             new TrapezoidProfile.Constraints(
  //                 Constants.Swerve.WHICH_SWERVE_ROBOT
  //                     .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
  //                     .maxVelocityLinear,
  //                 Constants.Swerve.WHICH_SWERVE_ROBOT
  //                     .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
  //                     .maxAccelerationLinear),
  //             new TrapezoidProfile.Constraints(
  //                 Constants.Swerve.WHICH_SWERVE_ROBOT
  //                     .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
  //                     .maxVelocityAngular,
  //                 Constants.Swerve.WHICH_SWERVE_ROBOT
  //                     .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
  //                     .maxAccelerationAngular)); // constants

  //     addRequirements(swerve);
  //   }

  /**
   * @param swerve Swerve Subsystem.
   * @param targetPoseSupplier Target Pose Supplier (for changing values of pose not just runtime)
   */
  public DriveToPose(CommandSwerveDrivetrain swerve, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.targetPose = targetPoseSupplier.get();

    Translation2d[] swerveModulePositions = new Translation2d[4];

    swerveModulePositions[0] =
        new Translation2d(
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2.0).magnitude(),
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2.0).magnitude());
    swerveModulePositions[1] =
        new Translation2d(
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(2.0).magnitude(),
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2.0).magnitude());
    swerveModulePositions[2] =
        new Translation2d(
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2.0).magnitude(),
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(2.0).magnitude());
    swerveModulePositions[3] =
        new Translation2d(
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.length.div(-2.0).magnitude(),
            Constants.Swerve.WHICH_SWERVE_ROBOT.ROBOT_DIMENSIONS.width.div(-2.0).magnitude());

    wheelForceCalculator =
        new WheelForceCalculator(swerveModulePositions, 58.967, 3.67); // constants

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

    previousTime = Utils.getCurrentTimeSeconds();
    prev = new ChassisSpeeds();

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
    double currTime = Utils.getCurrentTimeSeconds();
    double dt = currTime - previousTime;
    previousTime = currTime;

    if (pathState == null) return;

    pathState = path.calculate(currTime - startTime, pathState, targetPose);
    Pose2d pose = swerve.getCurrentState().Pose;
    Pose2d path = pathState.pose;

    // Generate the next speeds for the robot
    ChassisSpeeds targetSpeeds =
        new ChassisSpeeds(
            pathState.speeds.vxMetersPerSecond + xController.calculate(pose.getX(), path.getX()),
            pathState.speeds.vyMetersPerSecond + yController.calculate(pose.getY(), path.getY()),
            pathState.speeds.omegaRadiansPerSecond
                + headingController.calculate(
                    pose.getRotation().getRadians(), path.getRotation().getRadians()));

    //   feedforwards = null;
    if (dt > 0.0001) {
      feedforwards = wheelForceCalculator.calculate(dt, prev, targetSpeeds);
    }

    prev = targetSpeeds;

    // Apply the generated speeds
    swerve.applyFieldSpeeds(targetSpeeds, feedforwards);
    //   swerve.applyOneFieldSpeeds(targetSpeeds);
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
    // if (path.isFinished(Utils.getCurrentTimeSeconds() - startTime)) {
    //   return true;
    // }
    // return false;
    return atPosition();
  }
}
