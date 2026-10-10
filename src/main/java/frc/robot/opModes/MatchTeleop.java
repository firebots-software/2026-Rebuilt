package frc.robot.opModes;

import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.util.MiscUtils;
import org.wpilib.opmode.Teleop; // 2027 package namespace

@Teleop
public class MatchTeleop extends BaseOpMode {

  public MatchTeleop(Robot robot) {
    super(robot);
  }

  @Override
  public void start() {
    super.start();

    m_robotContainer.intakeSubsystem.applyBrakeConfigArm();

    DogLog.log("Elastic/FlashDriveConnected", MiscUtils.isFlashDriveConnected());
  }
}
