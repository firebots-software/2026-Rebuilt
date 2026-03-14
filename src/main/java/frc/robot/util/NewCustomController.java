package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewCustomController extends GenericHID {
  Trigger skib;
  Trigger reverseShoot;

  public NewCustomController(int port) {
    super(port);
    skib = new Trigger(() -> this.getRawButton(10));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
  }

  public Trigger Skib() {
    return skib;
  }

  public Trigger ReverseShoot() { return reverseShoot; }
}
