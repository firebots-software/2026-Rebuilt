package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewCustomController extends GenericHID {
  Trigger skib;
  Trigger reverseShoot, intakeOverride;

  public NewCustomController(int port) {
    super(port);
    skib = new Trigger(() -> this.getRawButton(10));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intakeOverride = new Trigger(() -> this.getRawButton(2));
  }

  public Trigger Skib() {
    return skib;
  }

  public Trigger ReverseShoot() {
    return reverseShoot;
  }

  public Trigger IntakeOverride() {
    return intakeOverride;
  }
}
