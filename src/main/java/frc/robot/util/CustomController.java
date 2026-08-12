package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CustomController extends GenericHID {
  private Trigger visionShootingLockout, intakeVisionLockout;
  private Trigger reverseShoot, intakeOverride;
  private Trigger outreachPreShoot;

  public CustomController(int port) {
    super(port);
    visionShootingLockout = new Trigger(() -> this.getRawButton(10));
    intakeVisionLockout = new Trigger(() -> this.getRawButton(11));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intakeOverride = new Trigger(() -> this.getRawButton(2));
    outreachPreShoot = new Trigger(() -> this.getRawButton(3));
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

  public Trigger outreachPreShoot() {
    return outreachPreShoot;
  }
}
