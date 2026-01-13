// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  private static double tolerance = 0.1;
  private final LoggedTalonFX motor1, motor2;
  private final LoggedTalonFX preShooterMotor;

  private float targetSpeed = 10f;

  private final DigitalInput beamBreak;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public ShooterSubsystem() {
    // curr limits, motor configs, motionmagic configs, sensor initialization
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);

    Slot0Configs s0c = new Slot0Configs().withKP(.4).withKI(.1).withKD(0);

    motor1 = new LoggedTalonFX(Constants.Shooter.motor1Constants.port);
    motor2 = new LoggedTalonFX(Constants.Shooter.motor2Constants.port);
    preShooterMotor =
        new LoggedTalonFX(Constants.Shooter.preShooterConstants.port);

    MotionMagicConfigs mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = targetSpeed;

    motor1.getConfigurator().apply(s0c);
    motor2.getConfigurator().apply(s0c);
    motor1.getConfigurator().apply(mmc);
    motor2.getConfigurator().apply(mmc);
    motor1.getConfigurator().apply(clc);
    motor2.getConfigurator().apply(clc);

    preShooterMotor.getConfigurator().apply(s0c);
    preShooterMotor.getConfigurator().apply(mmc);
    preShooterMotor.getConfigurator().apply(clc);

    motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    beamBreak = new DigitalInput(Constants.Shooter.ObjectDetectorPort);
  }

  // spin the shooter up to speed before the game piece goes through it
  public void rampUp() {
    motor1.setControl(new MotionMagicVelocityVoltage(targetSpeed));
  }

  public void runPreShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed * 4d);
    m_velocityControl.withFeedForward(0.1);
    preShooterMotor.setControl(m_velocityControl);
  }

  public void stopShooters() {
    motor1.setControl(new VelocityVoltage(0));
  }

  public void stopPreShooter() {
    preShooterMotor.setControl(new VelocityVoltage(0));
  }

  public void stopAll() {
    stopPreShooter();
    stopShooters();
  }

  // returns if your speed error is within the tolerance
  public boolean atSpeed() {
    return Math.abs(motor1.getVelocity().getValueAsDouble() - targetSpeed) <= tolerance;
  }

  // is beambreak sensor true/false
  public boolean objDetected() {
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    DogLog.log("ShooterSubsystem/Speed", motor1.getVelocity().getValueAsDouble());
    DogLog.log("ShooterSubsystem/AtSpeed", atSpeed());
    DogLog.log("ShooterSubsystem/ObjectDetected", objDetected());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
