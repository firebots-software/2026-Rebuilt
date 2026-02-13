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
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;
  private TalonFXSimState warmUpMotor1SimState, warmUpMotor2SimState, warmUpMotor3SimState;
  private DCMotorSim roller1Sim, roller2Sim, roller3Sim;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private double targetSpeed = 0;

  public ShooterSubsystem() {

    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_1_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_2_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_3_ID);

    Follower follower =
        new Follower(Constants.Shooter.WARMUP_MOTOR_1_ID, MotorAlignmentValue.Aligned);

    warmUpMotor2.setControl(follower);
    warmUpMotor3.setControl(follower);
    shooter = warmUpMotor1;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD)
            .withKV(Constants.Shooter.KV)
            .withKA(Constants.Shooter.KA);

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

    warmUpMotor1SimState = warmUpMotor1.getSimState();
    warmUpMotor2SimState = warmUpMotor2.getSimState();
    warmUpMotor3SimState = warmUpMotor3.getSimState();

    var krakenGearboxModel = DCMotor.getKrakenX60Foc(1);
    roller1Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel,
                Constants.Shooter.WARMUP_1_MOI_KG_M2,
                Constants.Shooter.MOTOR_ROTS_PER_WARMUP_1_ROTS),
            krakenGearboxModel);

    roller2Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel,
                Constants.Shooter.WARMUP_2_MOI_KG_M2,
                Constants.Shooter.MOTOR_ROTS_PER_WARMUP_2_ROTS),
            krakenGearboxModel);

    roller3Sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel,
                Constants.Shooter.WARMUP_3_MOI_KG_M2,
                Constants.Shooter.MOTOR_ROTS_PER_WARMUP_3_ROTS),
            krakenGearboxModel);

    warmUpMotor1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    warmUpMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    warmUpMotor3SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
  }

  public double calculateFtToRPS(double speed) {
    return speed
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER_IN * Math.PI / 12)
        * Constants.Shooter.MOTOR_ROTS_PER_SHOOTER_ROTS;
    // from surface speed in ft/sec to rps
  }

  public double calculateRPSToFt(double motorRPS) {
    // Convert motor RPS to shooter wheel RPS
    double shooterWheelRPS = motorRPS / Constants.Shooter.MOTOR_ROTS_PER_SHOOTER_ROTS;
    // Convert shooter wheel RPS to surface speed
    return shooterWheelRPS * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER_IN * Math.PI / 12);
  }

  // speed based on shooter wheel which is the one flinging the ball with a max of 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2 because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setSpeed(double speed) {
    targetSpeed = speed / 2;
    shooter.setControl(velocityRequest.withVelocity(calculateFtToRPS(targetSpeed)));
  }

  public void stop() {
    setSpeed(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(calculateFtToRPS(targetSpeed) - shooter.getVelocity().getValueAsDouble())
        <= Constants.Shooter.SHOOTER_SPEED_TOLERANCE_RPS;
  }

  public double getCurrentSpeed() {
    return calculateRPSToFt(shooter.getVelocity().getValueAsDouble());
  }

  // Comands
  public Command shootAtSpeedCommand() {
    return Commands.runEnd(() -> this.setSpeed(Constants.Shooter.SHOOT_FOR_AUTO), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("Doglog/shooter/targetSpeed", targetSpeed);
    DogLog.log("Doglog/shooter/isAtSpeed", isAtSpeed());
    DogLog.log("Doglog.shooter/currentSpeed", getCurrentSpeed());
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RobotController.getBatteryVoltage();
    warmUpMotor1SimState.setSupplyVoltage(batteryVoltage);
    warmUpMotor2SimState.setSupplyVoltage(batteryVoltage);
    warmUpMotor3SimState.setSupplyVoltage(batteryVoltage);

    double warmUp1Voltage = warmUpMotor1SimState.getMotorVoltage();
    double warmUp2Voltage = warmUpMotor2SimState.getMotorVoltage();
    double warmUp3Voltage = warmUpMotor3SimState.getMotorVoltage();

    roller1Sim.setInputVoltage(warmUp1Voltage);
    roller2Sim.setInputVoltage(warmUp2Voltage);
    roller3Sim.setInputVoltage(warmUp3Voltage);

    roller1Sim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    roller2Sim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    roller3Sim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    double warmUp1MotorRotPerSec = roller1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_1_ROTS;
    double warmUp2MotorRotPerSec = roller2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_2_ROTS;
    double warmUp3MotorRotPerSec = roller3Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_3_ROTS;

    warmUpMotor1SimState.setRotorVelocity(warmUp1MotorRotPerSec);
    warmUpMotor2SimState.setRotorVelocity(warmUp2MotorRotPerSec);
    warmUpMotor3SimState.setRotorVelocity(warmUp3MotorRotPerSec);

    double warmUpMotor1RotorPositionRotations =
        roller1Sim.getAngularPositionRotations() * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_1_ROTS;
    double warmUpMotor2RotorPositionRotations =
        roller2Sim.getAngularPositionRotations() * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_2_ROTS;

    double warmUpMotor3RotorPositionRotations =
        roller3Sim.getAngularPositionRotations() * Constants.Shooter.MOTOR_ROTS_PER_WARMUP_3_ROTS;

    warmUpMotor1SimState.setRawRotorPosition(warmUpMotor1RotorPositionRotations);
    warmUpMotor2SimState.setRawRotorPosition(warmUpMotor2RotorPositionRotations);
    warmUpMotor3SimState.setRawRotorPosition(warmUpMotor3RotorPositionRotations);
  }
}
