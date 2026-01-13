// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.NetworkChannel;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Velocity;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.MathUtils.Polynomials;
import frc.robot.MathUtils.Vector3;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private static double tolerance = 0.1;
  private final LoggedTalonFX motor1, motor2;

  private float targetSpeed = 0f;

  private final DigitalInput objSensor;

  public ShooterSubsystem() {
    Slot0Configs s0c = new Slot0Configs().withKP(.4).withKI(.1).withKD(0); 

    motor1 = new LoggedTalonFX(Constants.Shooter.motor1Constants.port);
    motor2 = new LoggedTalonFX(Constants.Shooter.motor2Constants.port);

    MotionMagicConfigs mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = targetSpeed;

    motor1.getConfigurator().apply(s0c);
    motor2.getConfigurator().apply(s0c);
    motor1.getConfigurator().apply(mmc);
    motor2.getConfigurator().apply(mmc);

    motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));

    objSensor = new DigitalInput(Constants.Shooter.ObjectDetectorPort);
  }

  public void shoot() {
    motor1.setControl(new MotionMagicVelocityVoltage(targetSpeed));
  }

  public void stop() {
    motor1.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {  
    return motor1.getVelocity().getValueAsDouble() - targetSpeed <= tolerance;
  }

  public boolean objDetected() {
    return objSensor.get();
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
