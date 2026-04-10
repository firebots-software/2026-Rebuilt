// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.PeterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

/**
 * Runs the intake and preshooter until IR sensor detects note
 *
 * @param subsystem The subsystem used by this command.
 */
public class RunIntakeUntilDetection extends Command {
  private PeterSubsystem peterSubsystem;

  public RunIntakeUntilDetection(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinUpIntake();
    peterSubsystem.spinUpPreShooterVelocity(); // Run Velocity instead of Voltage for intake
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    peterSubsystem.stopIntake();
    peterSubsystem.stopPreShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return peterSubsystem.notePresent();
  }
}
