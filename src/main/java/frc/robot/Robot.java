package frc.robot;

import org.wpilib.framework.OpModeRobot; // The correct 2027 OpMode entry point
import org.wpilib.command2.CommandScheduler;
import org.wpilib.system.RobotController;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import frc.robot.util.LoggedTalonFX;

public class Robot extends OpModeRobot { //
    private final RobotContainer m_robotContainer;

    public Robot() {
        // Global setup inside the constructor
        DogLog.setOptions(
            new DogLogOptions()
                .withCaptureDs(true)
                .withLogExtras(false)
                .withNtTunables(false)
        );
        RobotController.setBrownoutVoltages(6.0, 6.75);

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // m_robotContainer.visionPeriodic();
        CommandScheduler.getInstance().run();
        LoggedTalonFX.periodic_static();
    }

    public RobotContainer getContainer() {
        return m_robotContainer;
    }
}
