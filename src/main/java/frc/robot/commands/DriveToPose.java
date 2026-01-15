package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.LinearPath;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/** This Command drives the robot in a linear path to a specific pose. */
public class DriveToPose extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CommandSwerveDrivetrain swerve;

  // Initialize the LinearPath and the LinearPath.State to a null value
  private LinearPath path = null;
  private LinearPath.State pathState = null;

  // Initialize the Target Pose and the Target Pose Supplier to a null value
  private Pose2d targetPose = null;
  private Supplier<Pose2d> targetPoseSupplier = null;

  private final PIDController xController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(5, 0.0, 0.0);

  double currentTime = Utils.getCurrentTimeSeconds();

  /**
   * @param swerve Swerve Subsystem.
   * @param targetPose Target Pose (static).
   */
  public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.targetPose = targetPose;

    addRequirements(swerve);
  }

  /**
   * @param swerve Swerve Subsystem.
   * @param targetPoseSupplier Target Pose Supplier (for changing values of pose not just runtime)
   */
  public DriveToPose(CommandSwerveDrivetrain swerve, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.targetPose = targetPoseSupplier.get();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DogLog.log("Init Current Pose", swerve.getCurrentState().Pose);
    DogLog.log("Init Target Pose", targetPose);
    DogLog.log("Init Linear Path", path.toString());
    DogLog.log("Init Linear Path State", pathState.toString());
    DogLog.log("Init Target Pose", targetPose);
    DogLog.log("Init Target Pose Supplier", targetPoseSupplier.toString());

    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }

    path =
        new LinearPath(
            new TrapezoidProfile.Constraints(1, 1), new TrapezoidProfile.Constraints(0.2, 0.2));
    pathState =
        new LinearPath.State(swerve.getCurrentState().Pose, swerve.getCurrentState().Speeds);

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Utils.getCurrentTimeSeconds() - currentTime;

    if (pathState != null) {
      pathState = path.calculate(currTime, pathState, targetPose);

      // Generate the next speeds for the robot
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              pathState.speeds.vxMetersPerSecond
                  + xController.calculate(
                      swerve.getCurrentState().Pose.getX(), pathState.pose.getX()),
              pathState.speeds.vyMetersPerSecond
                  + yController.calculate(
                      swerve.getCurrentState().Pose.getY(), pathState.pose.getX()),
              pathState.speeds.omegaRadiansPerSecond
                  + headingController.calculate(
                      swerve.getCurrentState().Pose.getRotation().getRadians(),
                      pathState.pose.getRotation().getRadians()));

      // Apply the generated speeds
      swerve.applyFieldSpeeds(speeds);
    }

    DogLog.log("Current Pose", swerve.getCurrentState().toString());
    DogLog.log("Target Pose", targetPose);
    DogLog.log("Curr time", currTime);
    DogLog.log("Path created", path != null);
    DogLog.log("Path state", pathState != null);
    DogLog.log("Init Target Pose", targetPose);
    DogLog.log("Init Target Pose Supplier", targetPoseSupplier.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (path.isFinished(0)) {
    //   pathState = null;
    //   return true;
    // }
    // return false;
    return false;
  }
}
