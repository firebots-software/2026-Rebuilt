// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;
  private double targetShooterWheelSpeedRps = 0;

  // Simulation Objects
  private TalonFXSimState shooterSimState;
  private DCMotorSim shooterMechanismSim;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  public ShooterSubsystem() {
    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WARMUP_1_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WARMUP_2_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WARMUP_3_ID);
    shooter = warmUpMotor3;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.kP)
            .withKI(Constants.Shooter.kI)
            .withKD(Constants.Shooter.kD)
            .withKV(Constants.Shooter.kV)
            .withKA(Constants.Shooter.kA);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    VoltageConfigs voltageConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    // Apply full TalonFXConfiguration to ensure factory defaults
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = s0c;
    config.CurrentLimits = clc;
    config.MotorOutput = motorOutputConfigs;
    config.Voltage = voltageConfigs;

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(config);
    m2config.apply(config);
    m3config.apply(config);

    // Set motors 1 and 2 to follow motor 3 (the leader)
    Follower follower = new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);

    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.kP);
    DogLog.log("Subsystems/Shooter/Gains/kI", Constants.Shooter.kI);
    DogLog.log("Subsystems/Shooter/Gains/kD", Constants.Shooter.kD);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.kV);
    DogLog.log("Subsystems/Shooter/Gains/kA", Constants.Shooter.kA);

    if (RobotBase.isSimulation()) setupSimulation();
  }

  private void setupSimulation() {
    shooterSimState = warmUpMotor3.getSimState();
    shooterSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    shooterSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var singleKrakenGearbox = DCMotor.getKrakenX60Foc(1);

    shooterMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                singleKrakenGearbox,
                Constants.Shooter.Simulation.SHOOTER_SIM_MOI_KG_M2,
                Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT),
            singleKrakenGearbox);
  }

  // from linear speed in ft/sec to motor rps
  public double calculateFtPSToRPS(double speedFtPS) {
    return (speedFtPS * 12)
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI)
        * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT;
  }

  // from motor rps to linear speed in ft/sec
  public double calculateRPSToFtPS(double rps) {
    return rps
        * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI / 12)
        / Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT;
  }

  // TODO: Evaluate these old comments
  // speed based on shooter wheel which is the one flinging the ball with a max of 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2 because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setShooterWheelSpeedRps(double shooterSpeedRps) {
    targetShooterWheelSpeedRps = shooterSpeedRps;
    shooter.setControl(m_velocityRequest.withVelocity(targetShooterWheelSpeedRps * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT));
  }

  public void stopShooter() {
    setShooterWheelSpeedRps(0);
  }

  public boolean isAtSpeed() {
    return (Math.abs(targetShooterWheelSpeedRps - (shooter.getVelocity().getValueAsDouble() * Constants.Shooter.WHEEL_ROTS_PER_MOTOR_ROT))) < Constants.Shooter.WHEEL_TOLERANCE_RPS;
  }

  public double getCurrentWheelSpeedRps() {
    return shooter.getVelocity().getValueAsDouble();
  }

  // Commands that stop on end
  public Command shootAtSpeedUntilInterruptedCommand() {
    return runEnd(() -> setShooterWheelSpeedRps(Constants.Shooter.SHOOT_FOR_AUTO), this::stopShooter);
  }

  public Command shootAtSpeedUntilInterruptedCommand(double shooterWheelSpeed) {
    return runEnd(() -> setShooterWheelSpeedRps(shooterWheelSpeed), this::stopShooter);
  }

  public Command shootAtSpeedUntilInterruptedCommand(DoubleSupplier shooterWheelSpeedSupplier) {
    return runEnd(() -> this.setShooterWheelSpeedRps(shooterWheelSpeedSupplier.getAsDouble()), this::stopShooter);
  }

  // Same commands as above except they don't stop on end
  public Command shootAtSpeedCommand() {
    return runOnce(() -> setShooterWheelSpeedRps(Constants.Shooter.SHOOT_FOR_AUTO));
  }

  public Command shootAtSpeedCommand(double shooterWheelSpeed) {
    return runOnce(() -> setShooterWheelSpeedRps(shooterWheelSpeed));
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/CurrentWheelSpeed (rps)", getCurrentWheelSpeedRps());
    DogLog.log("Subsystems/Shooter/TargetWheelSpeed (rps)", targetShooterWheelSpeedRps);
    DogLog.log("Subsystems/Shooter/isAtSpeed", isAtSpeed());
  }

  @Override
  public void simulationPeriodic() {
    if (shooterSimState == null || shooterMechanismSim == null) {
      return;
    }

    // 1. How many volts applied to the motor?
    double batteryV = RobotController.getBatteryVoltage();
    shooterSimState.setSupplyVoltage(batteryV);
    double appliedMotorVoltageVolts =
        shooterSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    shooterMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
    shooterMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 2. What happens to the simulated mechanism?
    double shooterWheelVelocityRotationsPerSecond =
        shooterMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double shooterWheelPositionRotations = shooterMechanismSim.getAngularPositionRotations();

    // 3. Updating the simulated motor based on the behavior of the simulated mechanism
    double motorRotorPositionRotations =
        shooterWheelPositionRotations * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT;
    double motorRotorVelocityRotationsPerSecond =
        shooterWheelVelocityRotationsPerSecond * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROT;
    shooterSimState.setRawRotorPosition(motorRotorPositionRotations);
    shooterSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4. What happens to the battery (simulated)?
    double loadedBatteryVoltageVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSimState.getSupplyCurrent() * 3);
    RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);
  }
}
