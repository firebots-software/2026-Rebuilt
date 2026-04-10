package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmToAngleCmd extends Command {

  private final ArmSubsystem arm;
  private final Supplier<Double> angle;
  private double endToleranceDegrees = -1;
  private EndBehavior returnToRest = EndBehavior.RETURN_IF_INTERRUPTED;

  public enum EndBehavior {
    STAY,
    RETURN_ALWAYS,
    RETURN_IF_INTERRUPTED
  }

  public ArmToAngleCmd(Supplier<Double> angle, ArmSubsystem arm) {
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // SmartDashboard.putBoolean("RUNNNININGINNIG BUNDT", true);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("RUNNNINING Angle to target laowajoiadoijwjoiawd", angle.get());
    arm.setTargetDegrees(angle.get());
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget(endToleranceDegrees);
  }

  @Override
  public void end(boolean interrupted) {
    // SmartDashboard.putBoolean("RUNNNININGINNIG BUNDT", false);
    if (returnToRest == EndBehavior.RETURN_ALWAYS
        || (returnToRest == EndBehavior.RETURN_IF_INTERRUPTED && interrupted)) {
      arm.rotateToRestPosition();
    }
  }

  public ArmToAngleCmd withReturnToRest(EndBehavior v) {
    this.returnToRest = v;
    return this;
  }

  public ArmToAngleCmd withTolerance(double degrees) {
    this.endToleranceDegrees = degrees;
    return this;
  }

  public static ArmToAngleCmd toAmp(ArmSubsystem arm) {
    return new ArmToAngleCmd(() -> Constants.Arm.AMP_ANGLE, arm);
  }

  public static ArmToAngleCmd toNeutral(ArmSubsystem arm) {
    return new ArmToAngleCmd(() -> Constants.Arm.DEFAULT_ARM_ANGLE, arm);
  }

  public static ArmToAngleCmd toIntake(ArmSubsystem arm) {
    return new ArmToAngleCmd(() -> Constants.Arm.INTAKE_ANGLE, arm);
  }

  public static ArmToAngleCmd toBundt(ArmSubsystem arm) {
    return new ArmToAngleCmd(() -> Constants.Arm.BUNDT_ANGLE, arm);
  }

  public static ArmToAngleCmd toDuck(ArmSubsystem arm) {
    return new ArmToAngleCmd(() -> 12d, arm);
  }
}
