package frc.robot.commands;

import com.ctre.phoenix6.swerve.utility.LinearPath;
import edu.wpi.first.math.geometry.Pose2d;
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
    path =
        new LinearPath(
            new TrapezoidProfile.Constraints(1, 1), new TrapezoidProfile.Constraints(0.2, 0.2));
    pathState =
        new LinearPath.State(swerve.getCurrentState().Pose, swerve.getCurrentState().Speeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathState != null) {
      pathState = path.calculate(3.0, pathState, targetPose);

      swerve.applyFieldSpeeds(pathState.speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (path.isFinished(5)) {
      pathState = null;
      return true;
    }
    return false;
  }
}
