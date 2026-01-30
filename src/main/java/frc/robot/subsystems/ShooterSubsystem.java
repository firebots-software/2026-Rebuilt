// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private LoggedTalonFX motor1;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public ShooterSubsystem() {
    // Initialize motors
    motor1 =
        new LoggedTalonFX(
            "subsystems/Shooter/motor1",
            Constants.Shooter.MOTOR1_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    Slot0Configs shooterSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Shooter.KV)
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD);

    CurrentLimitsConfigs shooterCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator shooterMotor1Config = motor1.getConfigurator();

    shooterMotor1Config.apply(shooterSlot0Configs);
    shooterMotor1Config.apply(shooterCurrentLimitsConfigs);
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
