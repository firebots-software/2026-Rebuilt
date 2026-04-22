package frc.robot.commandGroups;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class LEDStrobeInRange extends SequentialCommandGroup {
  public LEDStrobeInRange(LEDSubsystem ledSubsystem) {
    addCommands(
        ledSubsystem.updateStateCommand(LEDState.ORANGE_SOLID),
        Commands.waitTime(Milliseconds.of(100)),
        ledSubsystem.updateStateCommand(LEDState.WHITE_SOLID),
        Commands.waitTime(Milliseconds.of(100)));
  }
}
