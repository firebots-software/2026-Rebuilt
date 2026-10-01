package frc.robot.opModes;

import org.wpilib.opmode.Autonomous; // 2027 OpMode namespace
import org.wpilib.command2.Command;   // 2027 Commands v2 package
import org.wpilib.command2.CommandScheduler; 
import frc.robot.Robot;
import frc.robot.util.VisionUtils;
import frc.robot.Constants;

@Autonomous
public class PrimaryAutonomous extends BaseOpMode {
    private Command m_autoCommand;

    public PrimaryAutonomous(Robot robot) {
        super(robot);
    }

    @Override
    public void start() {
        super.start();
        VisionUtils.setHeadingThreshold(Constants.Vision.MAX_HEADING_DIFF_AUTO);
        
        m_autoCommand = m_robotContainer.getAutonomousCommand();
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().schedule(m_autoCommand);
        }
    }

    @Override
    public void end() {
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().cancel(m_autoCommand);
        }
    }
}
