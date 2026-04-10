package frc.robot.commands.DebugCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.JoystickSubsystem;

public class Rumble extends Command {
  private JoystickSubsystem joystick;
  private final double intensity;

  public Rumble(JoystickSubsystem joystick, double intensity) {
    this.joystick = joystick;
    this.intensity = intensity;
    addRequirements(joystick);
  }

  @Override
  public void initialize() {
    joystick.rumble(intensity);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    joystick.rumble(0);
  }

  public static Command withNoBlock(
      JoystickSubsystem joystick, double intensity, double timeoutSeconds, double delaySeconds) {
    return new InstantCommand(
        () ->
            CommandScheduler.getInstance()
                .schedule(
                    new WaitCommand(delaySeconds)
                        .andThen(new Rumble(joystick, intensity).withTimeout(timeoutSeconds))));
  }
}
