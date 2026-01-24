// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmToAngle extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeSubsystem intakeSubsystem;

  private double position;

  public ArmToAngle(IntakeSubsystem subsystem, double position) {
    intakeSubsystem = subsystem;
    this.position = position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setArmDegrees(position);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
