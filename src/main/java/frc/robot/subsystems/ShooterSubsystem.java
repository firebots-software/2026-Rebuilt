// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private double targetSpeed = 0;
  private static final double TOLERANCE_RPS = 2.0; // tolerance in rotations per second

  // Simulation objects
  private TalonFXSimState motor1SimState;
  private TalonFXSimState motor2SimState;
  private TalonFXSimState motor3SimState;
  private DCMotorSim shooterMechanismSim;

  public ShooterSubsystem() {
    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WARMUP_1_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WARMUP_2_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WARMUP_3_ID);

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

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive);

    // Apply full TalonFXConfiguration to ensure factory defaults
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = s0c;
    config.CurrentLimits = clc;
    config.MotorOutput = motorOutputConfigs;

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(config);
    m2config.apply(config);
    m3config.apply(config);

    // Set motors 1 and 2 to follow motor 3 (the leader)
    Follower follower =
        new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    motor1SimState = warmUpMotor1.getSimState();
    motor2SimState = warmUpMotor2.getSimState();
    motor3SimState = warmUpMotor3.getSimState();

    motor1SimState.Orientation = ChassisReference.CounterClockwise_Positive;
    motor2SimState.Orientation = ChassisReference.CounterClockwise_Positive;
    motor3SimState.Orientation = ChassisReference.CounterClockwise_Positive;

    motor1SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    motor2SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    motor3SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Create a 3-motor gearbox model (all three motors working together)
    var threeKrakenGearbox = DCMotor.getKrakenX60Foc(3);

    shooterMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                threeKrakenGearbox,
                Constants.Shooter.SHOOTER_SIM_MOI_KG_M2,
                Constants.Shooter.SHOOTER_WHEEL_GEAR_RATIO),
            threeKrakenGearbox);
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
  public void setBallSpeed(double speed) {
    targetSpeed = speed / 2;
    warmUpMotor3.setControl(velocityRequest.withVelocity(calculateFtToRPS(targetSpeed)));
  }

  public void stop() {
    setBallSpeed(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(calculateFtToRPS(targetSpeed) - warmUpMotor3.getVelocity().getValueAsDouble())
        <= TOLERANCE_RPS;
  }

  public double getCurrentSpeed() {
    return calculateRPSToFt(warmUpMotor3.getVelocity().getValueAsDouble());
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return Commands.runEnd(() -> this.setBallSpeed(Constants.Shooter.SHOOT_FOR_AUTO), this::stop, this);
  }

  public Command shootAtSpeedCommand(double ballSpeed) {
    return Commands.runEnd(() -> this.setBallSpeed(ballSpeed), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/TargetSpeed", targetSpeed);
    DogLog.log("Subsystems/Shooter/IsAtSpeed", isAtSpeed());
    DogLog.log("Subsystems/Shooter/CurrentSpeed", getCurrentSpeed());
    DogLog.log("Subsystems/Shooter/Motor3VelocityRPS", warmUpMotor3.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/Motor3Volts", warmUpMotor3.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    if (motor1SimState == null || motor2SimState == null || motor3SimState == null || shooterMechanismSim == null) {
      return;
    }

    // 1) Supply voltage to all three motor sims
    double batteryV = RobotController.getBatteryVoltage();
    motor1SimState.setSupplyVoltage(batteryV);
    motor2SimState.setSupplyVoltage(batteryV);
    motor3SimState.setSupplyVoltage(batteryV);

    // 2) Read applied motor voltage from leader (motor3) and step mechanism plant
    // Since motor1 and motor2 follow motor3, we only read motor3's voltage
    double appliedMotorVoltageVolts =
        motor3SimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    
    shooterMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
    shooterMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 3) Mechanism-side sim -> rotor-side sensor state
    // DCMotorSim tracks the shooter wheel mechanism (after gear reduction)
    double shooterWheelVelocityRotationsPerSecond =
        shooterMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double shooterWheelPositionRotations = shooterMechanismSim.getAngularPositionRotations();

    // Convert mechanism rotations to motor rotor rotations
    double motorRotorPositionRotations =
        shooterWheelPositionRotations * Constants.Shooter.SHOOTER_WHEEL_GEAR_RATIO;
    double motorRotorVelocityRotationsPerSecond =
        shooterWheelVelocityRotationsPerSecond * Constants.Shooter.SHOOTER_WHEEL_GEAR_RATIO;

    // Update all three motor sim states with the same values (they're mechanically coupled)
    motor1SimState.setRawRotorPosition(motorRotorPositionRotations);
    motor1SimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);
    
    motor2SimState.setRawRotorPosition(motorRotorPositionRotations);
    motor2SimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);
    
    motor3SimState.setRawRotorPosition(motorRotorPositionRotations);
    motor3SimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4) Battery sag model
    // Sum the supply current from all three motors
    double totalSupplyCurrentAmps = 
        motor1SimState.getSupplyCurrent() + 
        motor2SimState.getSupplyCurrent() + 
        motor3SimState.getSupplyCurrent();

    double loadedBatteryVoltageVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(totalSupplyCurrentAmps);
    RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);
  }
}