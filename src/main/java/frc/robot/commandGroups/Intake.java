package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.ArmToAngle;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends ParallelCommandGroup {
  public Intake(IntakeSubsystem intake) {
    addCommands(new ArmToAngle(intake, Constants.Intake.Arm.armPosExtended), new RunIntake(intake));
  }
}
