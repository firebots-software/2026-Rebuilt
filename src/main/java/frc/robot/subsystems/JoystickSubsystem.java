package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickSubsystem extends SubsystemBase {
  private GenericHID joystick;

  public JoystickSubsystem(GenericHID joystick) {
    this.joystick = joystick;
  }

  public void rumble(double intensity) {
    joystick.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
  }
}
