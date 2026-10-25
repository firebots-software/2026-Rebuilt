package frc.robot.util;

import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.Trigger;

public class CustomController {
  private Trigger visionShootingLockout, intakeVisionLockout;
  private Trigger reverseShoot, intakeOverride;
  private final CommandGenericHID hid;

  public CustomController(int port) {
    hid = new CommandGenericHID(port);
    // super(port);
    visionShootingLockout = hid.button(10);
    intakeVisionLockout = hid.button(11);
    reverseShoot = hid.button(1);
    intakeOverride = hid.button(2);
  }

  public Trigger visionShootingLockout() {
    return visionShootingLockout;
  }

  public Trigger intakeVisionLockout() {
    return intakeVisionLockout;
  }

  public Trigger reverseShoot() {
    return reverseShoot;
  }

  public Trigger intakeOverride() {
    return intakeOverride;
  }
}
