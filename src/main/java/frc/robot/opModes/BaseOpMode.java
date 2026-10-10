package frc.robot.opModes;

import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.MiscUtils;
import frc.robot.util.TelemetryUtils;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.system.RobotController;

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
  public void periodic() {
    elasticLogging();
  }

  private void elasticLogging() {
    double matchTime = MatchState.getMatchTime();
    MiscUtils.shiftSwitchIndicator(matchTime);

    DogLog.log("Elastic/FieldPose", m_robotContainer.drivetrain.getCurrentState().Pose);
    DogLog.log("Elastic/BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.log("Elastic/AreWeActive", MiscUtils.areWeActive(matchTime));
    DogLog.log("Elastic/TimeUntilNextShift", MiscUtils.countdownTillNextShift(matchTime));
    DogLog.log("Elastic/CurrentShiftName", MiscUtils.currentShiftName(matchTime));

    TelemetryUtils.elasticTelemetry.log("CurrentShiftName", MiscUtils.currentShiftName(matchTime));
    TelemetryUtils.elasticTelemetry.log("ActiveFirst", MiscUtils.activeFirst());
    TelemetryUtils.elasticTelemetry.log("timeUntilNextShift", MiscUtils.countdownTillNextShift(matchTime));
  }
}