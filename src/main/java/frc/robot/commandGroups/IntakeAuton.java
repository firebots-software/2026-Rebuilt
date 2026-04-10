package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ArmToAngleCmd.EndBehavior;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.DebugCommands.Rumble;
import frc.robot.commands.PeterCommands.BackupPeter;
import frc.robot.commands.PeterCommands.RunIntakeUntilDetection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class IntakeAuton extends SequentialCommandGroup {
  public IntakeAuton(PeterSubsystem peter, ArmSubsystem arm, JoystickSubsystem joystick) {
    addCommands(
        new ResetArm(arm),
        new RunIntakeUntilDetection(peter)
            .deadlineFor(ArmToAngleCmd.toIntake(arm).withReturnToRest(EndBehavior.RETURN_ALWAYS)),
        new ParallelCommandGroup(
            // ArmToAngleCmd.toNeutral(arm).withTolerance(1),
            new BackupPeter(peter), Rumble.withNoBlock(joystick, 0.25, 0.5, 0)));
  }
}
