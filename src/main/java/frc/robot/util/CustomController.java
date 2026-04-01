package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController extends GenericHID {
  Trigger visionShootingLockout, intakeVisionLockout;
  Trigger reverseShoot, intakeOverride;

  public CustomController(int port) {
    super(port);
    visionShootingLockout = new Trigger(() -> this.getRawButton(10));
    intakeVisionLockout = new Trigger(() -> this.getRawButton(11));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intakeOverride = new Trigger(() -> this.getRawButton(2));
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
