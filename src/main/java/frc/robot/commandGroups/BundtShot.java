package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.ArmToAngleCmd;
import frc.robot.commands.ArmCommands.ResetArm;
import frc.robot.commands.DebugCommands.Rumble;
import frc.robot.commands.PeterCommands.ShootNoWarmup;
import frc.robot.commands.PeterCommands.SpinUpShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.PeterSubsystem;

public class BundtShot extends SequentialCommandGroup {
  public BundtShot(
      PeterSubsystem peterSubsystem,
      ArmSubsystem armSubsystem,
      JoystickSubsystem joystickSubsystem) {
    addCommands(
        new ResetArm(armSubsystem),
        new ParallelCommandGroup(
            new SpinUpShooter(peterSubsystem, false),
            ArmToAngleCmd.toBundt(armSubsystem).withTolerance(1)),
        new ParallelCommandGroup(
            new ShootNoWarmup(peterSubsystem, false).withTimeout(1),
            Rumble.withNoBlock(joystickSubsystem, 1, 1, 0.25),
            ArmToAngleCmd.toBundt(armSubsystem)));
  }
}
