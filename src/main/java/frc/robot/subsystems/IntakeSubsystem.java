// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;
  public LoggedTalonFX master;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public IntakeSubsystem() {
    // Initialize motors
    motor1 =
        new LoggedTalonFX(
            "subsystems/Intake/motor1",
            Constants.Intake.MOTOR1_PORT,
            "rio");
    motor2 =
        new LoggedTalonFX(
            "subsystems/Intake/motor1",
            Constants.Intake.MOTOR2_PORT,
            "rio");

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(Constants.Intake.MOTOR1_PORT, MotorAlignmentValue.Aligned);
    motor2.setControl(follower);
  }

  public void setProportion(double dutyCycleProportion) {
    motor1.setControl(dutyCycleRequest.withOutput(dutyCycleProportion));
  }

  @Override
  public void periodic() {
  }
}
