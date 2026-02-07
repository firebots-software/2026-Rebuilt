// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private static double targetSpeed = 0;
  private static double tolerance = 5; // rps

  // Simulation objects (3 motor-driven warmup rollers + 1 passive shooter roller)
  private TalonFXSimState warmUpMotor1SimState;
  private TalonFXSimState warmUpMotor2SimState;
  private TalonFXSimState warmUpMotor3SimState;

  private FlywheelSim warmUp1Sim;
  private FlywheelSim warmUp2Sim;
  private FlywheelSim warmUp3Sim;

  // Passive shooter wheel state (mechanically linked to warmUp1 via 18t -> 20t per MRD)
  private double shooterWheelPosRot = 0.0;
  private double shooterWheelVelRps = 0.0;

  private static final double SIM_DT_SEC = 0.020;

  // Estimated per-roller MOI for sim tuning
  private static final double WARMUP_ROLLER_MOI_KG_M2 = 0.0010;

  // Ratios from MRD (motor speed / roller speed)
  // motor->top: 16:18 = 0.8889 ; motor->mid: 16:24 = 0.6667 ; motor->bottom: 12:24 = 0.5
  private static final double MOTOR_TO_TOP_RATIO = 16.0 / 18.0;
  private static final double MOTOR_TO_MID_RATIO = 16.0 / 24.0;
  private static final double MOTOR_TO_BOTTOM_RATIO = 12.0 / 24.0;

  // Conversion used by setSpeed()/isAtSpeed() for final shooter wheel target
  // shooter : top = 20:18, so top is faster by 20/18
  private static final double TOP_OVER_SHOOTER = 20.0 / 18.0;

  public ShooterSubsystem() {

    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.warmUpMotor1.port);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.warmUpMotor2.port);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.warmUpMotor3.port);

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

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    warmUpMotor1SimState = warmUpMotor1.getSimState();
    warmUpMotor2SimState = warmUpMotor2.getSimState();
    warmUpMotor3SimState = warmUpMotor3.getSimState();

    warmUpMotor1SimState.Orientation = ChassisReference.CounterClockwise_Positive;
    warmUpMotor2SimState.Orientation = ChassisReference.CounterClockwise_Positive;
    warmUpMotor3SimState.Orientation = ChassisReference.CounterClockwise_Positive;

    warmUpMotor1SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    warmUpMotor2SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    warmUpMotor3SimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var kraken = DCMotor.getKrakenX60Foc(1);

    warmUp1Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kraken, WARMUP_ROLLER_MOI_KG_M2, MOTOR_TO_TOP_RATIO),
            kraken);
    warmUp2Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kraken, WARMUP_ROLLER_MOI_KG_M2, MOTOR_TO_MID_RATIO),
            kraken);
    warmUp3Sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                kraken, WARMUP_ROLLER_MOI_KG_M2, MOTOR_TO_BOTTOM_RATIO),
            kraken);
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
    // speed is requested shooter surface speed in ft/s
    targetSpeed = speed;

    // Convert requested shooter wheel surface speed -> shooter wheel rps
    double shooterRps = calculateFtToRPS(targetSpeed);

    // MRD: top warm-up runs faster than shooter by 20/18
    double topWarmupRpsTarget = shooterRps * TOP_OVER_SHOOTER;

    // Command only the top warm-up motor; other warm-up wheels reach speed through
    // their own closed loops/mechanical coupling in real hardware architecture.
    warmUpMotor1.setControl(velocityRequest.withVelocity(topWarmupRpsTarget));
  }

  public void stop() {
    setSpeed(0);
  }

  public boolean isAtSpeed() {
    double targetShooterRps = calculateFtToRPS(targetSpeed);
    return Math.abs(targetShooterRps - shooterWheelVelRps) <= tolerance;
  }

  public double getCurrentSpeed() {
    return calculateRPSToFt(shooterWheelVelRps);
  }

  // Comands
  public Command ShootAtSpeed() {
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
    if (warmUpMotor1SimState == null
        || warmUpMotor2SimState == null
        || warmUpMotor3SimState == null
        || warmUp1Sim == null
        || warmUp2Sim == null
        || warmUp3Sim == null) {
      return;
    }

    // 1) Provide battery voltage to all Talon sims
    double batteryV = RobotController.getBatteryVoltage();
    warmUpMotor1SimState.setSupplyVoltage(batteryV);
    warmUpMotor2SimState.setSupplyVoltage(batteryV);
    warmUpMotor3SimState.setSupplyVoltage(batteryV);

    // 2) Read applied voltages from each controller and step each warmup wheel plant
    double v1 = warmUpMotor1SimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double v2 = warmUpMotor2SimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double v3 = warmUpMotor3SimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

    warmUp1Sim.setInputVoltage(v1);
    warmUp2Sim.setInputVoltage(v2);
    warmUp3Sim.setInputVoltage(v3);

    warmUp1Sim.update(SIM_DT_SEC);
    warmUp2Sim.update(SIM_DT_SEC);
    warmUp3Sim.update(SIM_DT_SEC);

    // 3) Mechanism-side wheel state -> rotor-side Talon sensor state
    double warm1VelRps = warmUp1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double warm2VelRps = warmUp2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double warm3VelRps = warmUp3Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double warm1PosRot = warm1VelRps * SIM_DT_SEC;
    double warm2PosRot = warm2VelRps * SIM_DT_SEC;
    double warm3PosRot = warm3VelRps * SIM_DT_SEC;

    // Convert wheel speed to motor rotor speed via (motor/wheel) ratio
    double rotor1VelRps = warm1VelRps * MOTOR_TO_TOP_RATIO;
    double rotor2VelRps = warm2VelRps * MOTOR_TO_MID_RATIO;
    double rotor3VelRps = warm3VelRps * MOTOR_TO_BOTTOM_RATIO;

    double rotor1PosRot = warm1PosRot * MOTOR_TO_TOP_RATIO;
    double rotor2PosRot = warm2PosRot * MOTOR_TO_MID_RATIO;
    double rotor3PosRot = warm3PosRot * MOTOR_TO_BOTTOM_RATIO;

    warmUpMotor1SimState.addRotorPosition(rotor1PosRot);
    warmUpMotor2SimState.addRotorPosition(rotor2PosRot);
    warmUpMotor3SimState.addRotorPosition(rotor3PosRot);

    warmUpMotor1SimState.setRotorVelocity(rotor1VelRps);
    warmUpMotor2SimState.setRotorVelocity(rotor2VelRps);
    warmUpMotor3SimState.setRotorVelocity(rotor3VelRps);

    // 4) Passive shooter wheel is belt-coupled to top warm-up (18t -> 20t)
    shooterWheelVelRps = warm1VelRps * (18.0 / 20.0);
    shooterWheelPosRot += shooterWheelVelRps * SIM_DT_SEC;

    // 5) Battery sag from total current
    double totalCurrentAmps =
        warmUp1Sim.getCurrentDrawAmps() + warmUp2Sim.getCurrentDrawAmps() + warmUp3Sim.getCurrentDrawAmps();
    double loadedBatteryV = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
    RoboRioSim.setVInVoltage(loadedBatteryV);

    DogLog.log("Subsystems/Shooter/Sim/Warm1VelRps", warm1VelRps);
    DogLog.log("Subsystems/Shooter/Sim/Warm2VelRps", warm2VelRps);
    DogLog.log("Subsystems/Shooter/Sim/Warm3VelRps", warm3VelRps);
    DogLog.log("Subsystems/Shooter/Sim/ShooterWheelVelRps", shooterWheelVelRps);
    DogLog.log("Subsystems/Shooter/Sim/CurrentAmps", totalCurrentAmps);
    DogLog.log("Subsystems/Shooter/Sim/LoadedBatteryV", loadedBatteryV);
  }
}
