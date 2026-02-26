// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ZeroMuscleUp extends Command {
  private final ClimberSubsystem climberSubsystem;
  private double timesExceededCurrent = 0;

  public ZeroMuscleUp(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.reduceMuscleUpCurrentLimits();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.moveMuscleUpDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      climberSubsystem.resetMuscleUpPositionToZero();
    }
    climberSubsystem.resetMuscleUpCurrentLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climberSubsystem.checkMuscleUpCurrent()) {
      timesExceededCurrent++;
    } else {
      timesExceededCurrent = 0;
    }
    return timesExceededCurrent >= 10;
  }
}
