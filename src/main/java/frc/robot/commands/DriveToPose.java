// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.utility.LinearPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class DriveToPose extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CommandSwerveDrivetrain m_swerve;

  private LinearPath m_path = null;
  private LinearPath.State m_pathState = null;

  private Pose2d m_targetPose = null;
  private Supplier<Pose2d> m_targetPoseSupplier = null;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_targetPose = targetPose;

    addRequirements(swerve);
  }

  public DriveToPose(CommandSwerveDrivetrain swerve, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_targetPoseSupplier = targetPoseSupplier;
    m_targetPose = m_targetPoseSupplier.get();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_path =
        new LinearPath(
            new TrapezoidProfile.Constraints(1, 1), new TrapezoidProfile.Constraints(0.2, 0.2));
    m_pathState =
        new LinearPath.State(m_swerve.getCurrentState().Pose, m_swerve.getCurrentState().Speeds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pathState != null) {
      m_pathState = m_path.calculate(3.0, m_pathState, m_targetPose);

      m_swerve.applyFieldSpeeds(m_pathState.speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_path.isFinished(5)) {
      m_pathState = null;
      return true;
    }
    return false;
  }
}
