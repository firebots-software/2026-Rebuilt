package frc.robot.opModes;

import frc.robot.Robot;
import org.wpilib.command2.Command; // 2027 Commands v2 package
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous; // 2027 OpMode namespace

// @Autonomous
public class PrimaryAutonomous extends BaseOpMode {
  private Command m_autoCommand;

  public PrimaryAutonomous(Robot robot) {
    super(robot);
    m_autoCommand = m_robotContainer.getAutonomousCommand();

  }

  @Override
  public void start() {
    super.start();

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
