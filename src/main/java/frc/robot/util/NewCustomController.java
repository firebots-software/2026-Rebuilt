package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewCustomController extends GenericHID {
  Trigger skib;

  public NewCustomController(int port) {
    super(port);
    skib = new Trigger(() -> this.getRawButton(9));
  }

  public Trigger Skib() {
    return skib;
  }
}
