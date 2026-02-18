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

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private double targetBallSpeed = 0; // this needs to be consistent
  private static final double TOLERANCE_RPS = 2.0; // tolerance in rotations per second

  // Simulation objects
  private TalonFXSimState shooterSimState;
  private DCMotorSim shooterMechanismSim;

  public ShooterSubsystem() {
    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WARMUP_1_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WARMUP_2_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WARMUP_3_ID);
    shooter = warmUpMotor3;

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
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

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
    Follower follower = new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);

    if (RobotBase.isSimulation()) setupSimulation();
  }

  private void setupSimulation() {
    shooterSimState = warmUpMotor3.getSimState();
    shooterSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    shooterSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Use a SINGLE motor model since only Motor 3 is actively controlled
    var singleKrakenGearbox = DCMotor.getKrakenX60Foc(1);

    shooterMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                singleKrakenGearbox,
                Constants.Shooter.SHOOTER_SIM_MOI_KG_M2, // MOI of entire coupled system
                Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS), // Motor 3 → Shooter wheel (1.25)
            singleKrakenGearbox);
  }

  // from linear speed in ft/sec to motor rps
  public double calculateFtPSToRPS(double speedFtPS) {
    return (speedFtPS * 12)
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI)
        * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
  }

  // from motor rps to linear speed in ft/sec
  public double calculateRPSToFtPS(double rps) {
    return rps
        * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI / 12)
        / Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
  }

  // speed based on shooter wheel which is the one flinging the ball with a max of 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2 because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setBallSpeed(double ballSpeed) {
    targetBallSpeed = ballSpeed;
    shooter.setControl(m_velocityRequest.withVelocity(calculateFtPSToRPS(targetBallSpeed / 2.0)));
  }

  public void stopShooter() {
    setBallSpeed(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(
            calculateFtPSToRPS(targetBallSpeed) - (shooter.getVelocity().getValueAsDouble() * 2))
        <= TOLERANCE_RPS;
  }

  public double getCurrentBallSpeed() {
    return calculateRPSToFtPS(shooter.getVelocity().getValueAsDouble()) * 2;
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setBallSpeed(Constants.Shooter.SHOOT_FOR_AUTO), this::stopShooter);
  }

  public Command shootAtSpeedCommand(double ballSpeed) {
    return runEnd(() -> setBallSpeed(ballSpeed), this::stopShooter);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/targetSpeed", targetBallSpeed);
    DogLog.log("Subsystems/Shooter/isAtSpeed", isAtSpeed());
    DogLog.log("Subsystems/Shooter/currentSpeed", getCurrentBallSpeed());
    DogLog.log(
        "Subsystems/Shooter/motor3VelocityRPS", warmUpMotor3.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/motor3Volts", warmUpMotor3.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    if (shooterSimState == null || shooterMechanismSim == null) {
      return;
    }

    // 1) Supply voltage to all three motor sims
    double batteryV = RobotController.getBatteryVoltage();
    shooterSimState.setSupplyVoltage(batteryV);

    // 2) Read applied motor voltage from leader (motor3) and step mechanism plant
    // Since motor1 and motor2 follow motor3, we only read motor3's voltage
    double appliedMotorVoltageVolts =
        shooterSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

    shooterMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
    shooterMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 3) Mechanism-side sim -> rotor-side sensor state
    // DCMotorSim tracks the shooter wheel mechanism (after gear reduction)
    double shooterWheelVelocityRotationsPerSecond =
        shooterMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double shooterWheelPositionRotations = shooterMechanismSim.getAngularPositionRotations();

    // Convert mechanism rotations to motor rotor rotations
    double motorRotorPositionRotations =
        shooterWheelPositionRotations * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
    double motorRotorVelocityRotationsPerSecond =
        shooterWheelVelocityRotationsPerSecond * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;

    shooterSimState.setRawRotorPosition(motorRotorPositionRotations);
    shooterSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4) Battery sag model
    // Sum the supply current from all three motors
    double loadedBatteryVoltageVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSimState.getSupplyCurrent() * 3);
    RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);
  }
}
