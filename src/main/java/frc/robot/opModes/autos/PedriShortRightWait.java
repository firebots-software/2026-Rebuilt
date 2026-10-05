package frc.robot.opModes.autos;

import frc.robot.Constants.Swerve.Auto.AutoList;
import frc.robot.Robot;
import frc.robot.opModes.BaseOpMode;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.Autonomous;

@Autonomous
public class PedriShortRightWait extends BaseOpMode {
  private Command m_autoCommand;

  public PedriShortRightWait(Robot robot) {
    super(robot);
  }

  @Override
  public void start() {
    super.start();
    m_autoCommand = m_robotContainer.getAutonomousCommand(AutoList.RIGHT_WAIT);
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
