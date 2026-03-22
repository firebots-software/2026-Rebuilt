package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController extends GenericHID {
  Trigger skib, reverseShoot, intakeOverride;

  public CustomController(int port) {
    super(port);
    skib = new Trigger(() -> this.getRawButton(10));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intakeOverride = new Trigger(() -> this.getRawButton(2));
  }

  public Trigger skib() {
    return skib;
  }

  public Trigger reverseShoot() {
    return reverseShoot;
  }

  public Trigger intakeOverride() {
    return intakeOverride;
  }
}
