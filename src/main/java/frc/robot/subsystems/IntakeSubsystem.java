// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    motor2 =
        new LoggedTalonFX(
            "subsystems/Intake/motor1",
            Constants.Intake.MOTOR2_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    Slot0Configs intakeSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.KV)
            .withKP(Constants.Intake.KP)
            .withKI(Constants.Intake.KI)
            .withKD(Constants.Intake.KD);

    CurrentLimitsConfigs intakeCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Intake.INTAKE_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.INTAKE_SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator intakeMotor1Config = motor1.getConfigurator();
    TalonFXConfigurator intakeMotor2Config = motor2.getConfigurator();

    intakeMotor1Config.apply(intakeSlot0Configs);
    intakeMotor2Config.apply(intakeCurrentLimitsConfigs);

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(Constants.Intake.MOTOR1_PORT, MotorAlignmentValue.Aligned);
    motor2.setControl(follower);
  }

  public void setProportion(double dutyCycleProportion) {
    motor1.setControl(dutyCycleRequest.withOutput(dutyCycleProportion));
  }

  public void setVelocity(double motorVelocity) {
    motor1.setControl(velocityRequest.withVelocity(motorVelocity));
  }

  // Commands
  public Command runIntakeDutyCycle(double dutyCycleProportion) {
    return Commands.startEnd(
        () -> this.setProportion(dutyCycleProportion), () -> this.setProportion(0), this);
  }

  public Command runIntakeVelocity(double velocity) {
    return Commands.startEnd(() -> this.setVelocity(velocity), () -> this.setVelocity(0), this);
  }

  @Override
  public void periodic() {}
}
