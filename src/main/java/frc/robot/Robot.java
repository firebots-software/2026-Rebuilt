// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTalonFX;
import frc.robot.util.MiscUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    DogLog.setOptions(
        new DogLogOptions().withNtPublish(true).withCaptureDs(true).withLogExtras(false));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    LoggedTalonFX.periodic_static();
    m_robotContainer.visionPeriodic();

    MiscUtils.shiftSwitchIndicator();
    elasticLogging();
  }

  private void elasticLogging() {
    DogLog.log("Power/BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.log("Elastic/areWeActive", MiscUtils.areWeActive());
    DogLog.log("Elastic/timeUntilNextShift", MiscUtils.countdownTillNextShift());
    DogLog.log("Elastic/currentShiftName", MiscUtils.currentShiftName());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // schedule the autonomous command
    if (m_autonomousCommand != null)
      CommandScheduler.getInstance().schedule(m_robotContainer.getAutonomousCommand());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.intakeSubsystem.applyBrakeConfigArm();
    // cancel auto commands when teleop starts
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();

    DogLog.log("Elastic/FlashDriveConnected", MiscUtils.isFlashDriveConnected());
  }

  @Override
  public void teleopPeriodic() {}
}
