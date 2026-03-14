package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NewCustomController extends GenericHID {
  Trigger skib;
  Trigger intermapDown, intermapUp, intermapShoot;
  Trigger reverseShoot;

  public NewCustomController(int port) {
    super(port);
    skib = new Trigger(() -> this.getRawButton(10));
    reverseShoot = new Trigger(() -> this.getRawButton(1));
    intermapDown = new Trigger(() -> this.getRawButton(9));
    intermapUp = new Trigger(() -> this.getRawButton(7));
    intermapShoot = new Trigger(() -> this.getRawButton(8));
  }

  public Trigger Skib() {
    return skib;
  }

  public Trigger IntermapDown() {
    return intermapDown;
  }

  public Trigger IntermapUp() {
    return intermapUp;
  }

  public Trigger IntermapShoot() {
    return intermapShoot; }
  public Trigger ReverseShoot() {
    return reverseShoot;
  }
}
