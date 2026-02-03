// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  private static double tolerance = 1;
  private final LoggedTalonFX motor1, motor2;
  private final LoggedTalonFX preShooterMotor;
  private final double wheelradius = 2d / 12d / 3d;

  private double targetSpeed = 0;

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

    Slot0Configs s0c = new Slot0Configs().withKP(.4).withKI(0).withKD(0).withKV(0.1185);
    Slot0Configs psS0c = new Slot0Configs().withKP(.1).withKI(0).withKD(0).withKV(0.1185);

    motor1 = new LoggedTalonFX(Constants.Shooter.motor1Constants.port);
    motor2 = new LoggedTalonFX(Constants.Shooter.motor2Constants.port);
    preShooterMotor = new LoggedTalonFX(Constants.Shooter.preShooterConstants.port);

    motor1.getConfigurator().apply(s0c);
    motor2.getConfigurator().apply(s0c);
    motor1.getConfigurator().apply(clc);
    motor2.getConfigurator().apply(clc);

    preShooterMotor.getConfigurator().apply(psS0c);
    preShooterMotor.getConfigurator().apply(clc);

    motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  // spin the shooter up to speed before the game piece goes through it
  public void rampUp(double speed) {
    targetSpeed = speed;
    VelocityVoltage m_velocityControl =
        new VelocityVoltage(speed * (24d / 18d) / (2 * 3.14 * wheelradius));
    motor1.setControl(m_velocityControl);
  }

  public void runPreShooterAtRPS(double speed) {
    VelocityVoltage m_velocityControl = new VelocityVoltage(speed * -4d);
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
    return Math.abs(
            motor1.getVelocity().getValueAsDouble()
                - targetSpeed * (24d / 18d) / (2 * 3.14 * wheelradius))
        <= tolerance;
  }

  @Override
  public void periodic() {
    DogLog.log(
        "ShooterSubsystem/MotorSpeed",
        motor1.getVelocity().getValueAsDouble());
    DogLog.log("ShooterSubsystem/PSSpeed", preShooterMotor.getVelocity().getValueAsDouble());
    DogLog.log("ShooterSubsystem/AtSpeed", atSpeed());
    DogLog.log("ShooterSubsystem/TargetSpeed", targetSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
