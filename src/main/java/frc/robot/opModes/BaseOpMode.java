package frc.robot.opModes;

import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.MiscUtils;
import frc.robot.util.TelemetryUtils;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.system.RobotController;

public abstract class BaseOpMode extends PeriodicOpMode {
  protected final RobotContainer m_robotContainer;
  private static double simulatedTime = 160;

  public BaseOpMode(Robot robot) {
    this.m_robotContainer = robot.getContainer();
  }

  @Override
  public void start() {
    // Shared configuration when the OpMode begins
    DogLog.log("Elastic/FieldPose", m_robotContainer.drivetrain.getCurrentState().Pose);
    DogLog.log("Elastic/RedSide", RobotContainer.isRedAlliance());
    // VisionUtils.setHeadingThreshold(Constants.Vision.MAX_HEADING_DIFF_AUTO);
  }

  @Override
  public void periodic() {
    // m_robotContainer.visionPeriodic();
    // Handled every 20ms during execution
    elasticLogging();
    MiscUtils.shiftSwitchIndicator(simulatedTime);
  }

  private void elasticLogging() {
    simulatedTime -= 0.02;
    if (simulatedTime < 0) simulatedTime = 160;

    DogLog.log("Elastic/FieldPose", m_robotContainer.drivetrain.getCurrentState().Pose);
    DogLog.log("Elastic/BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.log("Elastic/AreWeActive", MiscUtils.areWeActive());
    DogLog.log("Elastic/TimeUntilNextShift", MiscUtils.countdownTillNextShift(simulatedTime));
    DogLog.log("Elastic/CurrentShiftName", MiscUtils.currentShiftName(simulatedTime));

    TelemetryUtils.elasticTelemetry.log(
        "CurrentShiftName", MiscUtils.currentShiftName(simulatedTime));
    TelemetryUtils.elasticTelemetry.log("ActiveFirst", MiscUtils.activeFirst());
    TelemetryUtils.elasticTelemetry.log(
        "timeUntilNextShift", MiscUtils.countdownTillNextShift(simulatedTime));
  }
}
