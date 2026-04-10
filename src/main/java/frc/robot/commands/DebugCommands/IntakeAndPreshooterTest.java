// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PeterSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeAndPreshooterTest extends Command {
  private PeterSubsystem peterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param peterSubsystem The subsystem used by this command.
   */
  public IntakeAndPreshooterTest(PeterSubsystem peterSubsystem) {
    this.peterSubsystem = peterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(peterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    peterSubsystem.spinUpIntake();
    peterSubsystem.spinUpPreShooterVoltage();
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
    return false; // wait for x seconds
  }
}
