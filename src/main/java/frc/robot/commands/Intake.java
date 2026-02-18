package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends Command {
  private IntakeSubsystem intakeSubsystem;

  public Intake(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    intakeSubsystem.setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
    intakeSubsystem.runRollers(Constants.Intake.Rollers.TARGET_ROLLER_RPS);
  }

  @Override
  public void execute() {
    intakeSubsystem.setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopRollers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
