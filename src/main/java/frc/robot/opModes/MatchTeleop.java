package frc.robot.opModes;

import org.wpilib.opmode.Teleop; // 2027 package namespace
import dev.doglog.DogLog;
import frc.robot.Robot;
import frc.robot.util.MiscUtils;
import frc.robot.util.VisionUtils;
import frc.robot.Constants;

@Teleop
public class MatchTeleop extends BaseOpMode {

    public MatchTeleop(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();
        
        m_robotContainer.intakeSubsystem.applyBrakeConfigArm();
        VisionUtils.setHeadingThreshold(Constants.Vision.MAX_HEADING_DIFF);
        
        DogLog.log("Elastic/FlashDriveConnected", MiscUtils.isFlashDriveConnected());
    }
}
