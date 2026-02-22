package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends ParallelCommandGroup {
  public RetractIntake(IntakeSubsystem intake) {
    addCommands(intake.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }
}
