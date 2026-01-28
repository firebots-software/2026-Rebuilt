// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, master;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(null);

  private static double targetSpeed = 0;
  private static double tolerance = 5; // rps

  public ShooterSubsystem() {

    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.warmUpMotor1.port);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.warmUpMotor2.port);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.warmUpMotor3.port);

    Follower follower =
        new Follower(Constants.Shooter.warmUpMotor1.port, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);
    warmUpMotor3.setControl(follower);
    master = warmUpMotor1;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.SHOOTER_KP)
            .withKI(Constants.Shooter.SHOOTER_KI)
            .withKD(Constants.Shooter.SHOOTER_KD)
            .withKV(Constants.Shooter.SHOOTER_KV)
            .withKA(Constants.Shooter.SHOOTER_KA);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(s0c);
    m2config.apply(s0c);
    m3config.apply(s0c);
    m1config.apply(clc);
    m2config.apply(clc);
    m3config.apply(clc);
  }

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public double calculateFtToRPS(double speed) {
    return speed
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI / 12)
        * Constants.Shooter.SHOOTER_WHEEL_GEAR_RATIO;
    // from surface speed in ft/sec to rps
  }

  public double calculateRPSToFt(double rps) {
    return rps
        * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI / 12)
        / Constants.Shooter.SHOOTER_WHEEL_GEAR_RATIO;
  }

  // speed based on shooter wheel which is the one flinging the ball with a max of 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2 because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setSpeed(double speed) {
    targetSpeed = speed / 2;
    master.setControl(velocityRequest.withVelocity(calculateFtToRPS(targetSpeed)));
  }

  public void stop() {
    setSpeed(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(calculateFtToRPS(targetSpeed) - master.getVelocity().getValueAsDouble())
        <= tolerance;
  }

  public double getCurrentSpeed() {
    return calculateRPSToFt(master.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    DogLog.log("Doglog/shooter/targetSpeed", targetSpeed);
    DogLog.log("Doglog/shooter/isAtSpeed", isAtSpeed());
    DogLog.log("Doglog.shooter/currentSpeed", getCurrentSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
