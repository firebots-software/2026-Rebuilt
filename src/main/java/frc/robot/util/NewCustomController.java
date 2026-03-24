package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewCustomController extends GenericHID {
  Trigger visionShootingLockout, intakeVisionLockout;
  Trigger reverseShoot, intakeOverride;

  public NewCustomController(int port) {
    super(port);
    visionShootingLockout = new Trigger(() -> this.getRawButton(10));
    intakeVisionLockout = new Trigger(() -> this.getRawButton(11));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intakeOverride = new Trigger(() -> this.getRawButton(2));
  }

  public Trigger VisionShootingLockout() {
    return visionShootingLockout;
  }

  public Trigger IntakeVisionLockout() {
    return intakeVisionLockout;
  }

  public Trigger ReverseShoot() {
    return reverseShoot;
  }

  public Trigger IntakeOverride() {
    return intakeOverride;
  }
}
