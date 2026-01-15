// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class RunIntakeUntilDetection extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RunIntakeUntilDetection(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    intakeSubsystem.run(100);
    shooterSubsystem.runPreShooterAtRPS(20);
  }

  
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
    shooterSubsystem.stopPreShooter();
  }

  
  @Override
  public boolean isFinished() {
    return intakeSubsystem.beamBroken();
  }
}
