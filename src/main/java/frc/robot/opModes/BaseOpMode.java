package frc.robot.opModes;

import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import org.wpilib.opmode.PeriodicOpMode;

public abstract class BaseOpMode extends PeriodicOpMode {
  protected final RobotContainer m_robotContainer;

  public BaseOpMode(Robot robot) {
    this.m_robotContainer = robot.getContainer();
  }

  @Override
  public void start() {
    DogLog.log("Elastic/FieldPose", m_robotContainer.drivetrain.getCurrentState().Pose);
    DogLog.log("Elastic/RedSide", RobotContainer.isRedAlliance());
  }

  @Override
  public void periodic() {}
}
