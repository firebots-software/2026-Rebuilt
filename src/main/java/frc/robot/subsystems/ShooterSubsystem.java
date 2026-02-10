// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  // Three independent motors, each driving its own warm-up roller
  private final LoggedTalonFX warmUpMotor1; // Top roller (fastest)
  private final LoggedTalonFX warmUpMotor2; // Middle roller
  private final LoggedTalonFX warmUpMotor3; // Bottom roller (slowest)

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private static double targetShooterSurfaceSpeed = 0; // ft/s
  private static double tolerance = 5; // rps

  // Simulation objects
  private final TalonFXSimState warmUpMotor1SimState;
  private final TalonFXSimState warmUpMotor2SimState;
  private final TalonFXSimState warmUpMotor3SimState;
  
  // Individual flywheel simulations for each roller
  private final DCMotorSim warmUp1Sim;
  private final DCMotorSim warmUp2Sim;
  private final DCMotorSim warmUp3Sim;

  public ShooterSubsystem() {
    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WarmUp1.CAN_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WarmUp2.CAN_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WarmUp3.CAN_ID);

    // Configure PID and feedforward for all motors
    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD)
            .withKV(Constants.Shooter.KV)
            .withKA(Constants.Shooter.KA);

    // Configure current limits
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);

    // Apply configurations to all motors
    TalonFXConfigurator config1 = warmUpMotor1.getConfigurator();
    TalonFXConfigurator config2 = warmUpMotor2.getConfigurator();
    TalonFXConfigurator config3 = warmUpMotor3.getConfigurator();

    config1.apply(slot0);
    config2.apply(slot0);
    config3.apply(slot0);
    
    config1.apply(currentLimits);
    config2.apply(currentLimits);
    config3.apply(currentLimits);

    // Initialize simulation objects
    warmUpMotor1SimState = warmUpMotor1.getSimState();
    warmUpMotor2SimState = warmUpMotor2.getSimState();
    warmUpMotor3SimState = warmUpMotor3.getSimState();

    var krakenGearboxModel = DCMotor.getKrakenX60Foc(1);
    // Create flywheel simulations for each roller
    warmUp1Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(krakenGearboxModel, Constants.Shooter.WarmUp1.MOI_KG_M2, 1.0 / Constants.Shooter.WarmUp1.MOTOR_ROTS_PER_ROLLER_ROTS), krakenGearboxModel);
    warmUp2Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(krakenGearboxModel, Constants.Shooter.WarmUp2.MOI_KG_M2, 1.0 / Constants.Shooter.WarmUp2.MOTOR_ROTS_PER_ROLLER_ROTS), krakenGearboxModel);
    warmUp3Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(krakenGearboxModel, Constants.Shooter.WarmUp3.MOI_KG_M2, 1.0 / Constants.Shooter.WarmUp3.MOTOR_ROTS_PER_ROLLER_ROTS), krakenGearboxModel);

    // Set simulation initial voltages
    warmUpMotor1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    warmUpMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    warmUpMotor3SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
  }

  /**
   * Convert shooter wheel surface speed (ft/s) to shooter wheel RPS
   */
  private double shooterSurfaceSpeedToShooterWheelRPS(double surfaceSpeedFtPerSec) {
    // Surface speed = wheel circumference * RPS
    // RPS = surface speed / circumference
    double circumferenceFt = Constants.Shooter.ShooterWheel.DIAMETER_INCHES * Math.PI / 12.0;
    return surfaceSpeedFtPerSec / circumferenceFt;
  }

  /**
   * Convert shooter wheel RPS to surface speed (ft/s)
   */
  private double shooterWheelRPSToSurfaceSpeed(double shooterWheelRPS) {
    double circumferenceFt = Constants.Shooter.ShooterWheel.DIAMETER_INCHES * Math.PI / 12.0;
    return shooterWheelRPS * circumferenceFt;
  }

  /**
   * Calculate motor velocities needed to achieve target shooter wheel speed.
   * Works backwards through the mechanical linkages.
   */
  private static class MotorSpeeds {
    double warmUp1MotorRPS;
    double warmUp2MotorRPS;
    double warmUp3MotorRPS;
  }

  private MotorSpeeds calculateMotorSpeeds(double shooterWheelRPS) {
    MotorSpeeds speeds = new MotorSpeeds();

    // Shooter wheel is driven by warmUp1 through 18t:20t belt
    // If shooter needs X RPS, warmUp1 roller needs X * (20/18) RPS
    double warmUp1RollerRPS = shooterWheelRPS * Constants.Shooter.BeltRatios.SHOOTER_TO_WARMUP1;
    
    // WarmUp1 motor drives roller through motor:roller gear ratio
    speeds.warmUp1MotorRPS = warmUp1RollerRPS * Constants.Shooter.WarmUp1.MOTOR_ROTS_PER_ROLLER_ROTS;

    // WarmUp2 roller is driven by warmUp1 through 15t:20t belt
    // If warmUp1 needs X RPS, warmUp2 needs X * (20/15) RPS
    double warmUp2RollerRPS = warmUp1RollerRPS * Constants.Shooter.BeltRatios.WARMUP1_TO_WARMUP2;
    speeds.warmUp2MotorRPS = warmUp2RollerRPS * Constants.Shooter.WarmUp2.MOTOR_ROTS_PER_ROLLER_ROTS;

    // WarmUp3 roller is driven by warmUp2 through 15t:20t belt
    double warmUp3RollerRPS = warmUp2RollerRPS * Constants.Shooter.BeltRatios.WARMUP2_TO_WARMUP3;
    speeds.warmUp3MotorRPS = warmUp3RollerRPS * Constants.Shooter.WarmUp3.MOTOR_ROTS_PER_ROLLER_ROTS;

    return speeds;
  }

  /**
   * Set shooter speed based on desired surface speed of the shooter wheel (ft/s).
   * The speed is divided by 2 as per Jeff's recommendation.
   * 
   * @param surfaceSpeedFtPerSec Desired surface speed (max: 52.36, min: 35.60 ft/s)
   *                             Actual input range is doubled (max: 104.72, min: 71.2)
   */
  public void setSpeed(double surfaceSpeedFtPerSec) {
    // Jeff's recommendation: divide input by 2
    targetShooterSurfaceSpeed = surfaceSpeedFtPerSec / 2.0;
    
    // Convert to shooter wheel RPS
    double shooterWheelRPS = shooterSurfaceSpeedToShooterWheelRPS(targetShooterSurfaceSpeed);
    
    // Calculate motor speeds needed
    MotorSpeeds speeds = calculateMotorSpeeds(shooterWheelRPS);
    
    // Command each motor independently
    warmUpMotor1.setControl(velocityRequest.withVelocity(speeds.warmUp1MotorRPS));
  }

  public void stop() {
    warmUpMotor1.setControl(velocityRequest.withVelocity(0));
    targetShooterSurfaceSpeed = 0;
  }

  /**
   * Check if all motors are at their target speeds within tolerance
   */
  public boolean isAtSpeed() {
    double targetShooterWheelRPS = shooterSurfaceSpeedToShooterWheelRPS(targetShooterSurfaceSpeed);
    MotorSpeeds targetSpeeds = calculateMotorSpeeds(targetShooterWheelRPS);
    
    boolean motor1AtSpeed = Math.abs(targetSpeeds.warmUp1MotorRPS - warmUpMotor1.getVelocity().getValueAsDouble()) <= tolerance;
    boolean motor2AtSpeed = Math.abs(targetSpeeds.warmUp2MotorRPS - warmUpMotor2.getVelocity().getValueAsDouble()) <= tolerance;
    boolean motor3AtSpeed = Math.abs(targetSpeeds.warmUp3MotorRPS - warmUpMotor3.getVelocity().getValueAsDouble()) <= tolerance;
    
    return motor1AtSpeed && motor2AtSpeed && motor3AtSpeed;
  }

  /**
   * Get current shooter surface speed based on warmUp1 motor velocity
   */
  public double getCurrentSpeed() {
    // Work forward from motor1 velocity to shooter surface speed
    double motor1RPS = warmUpMotor1.getVelocity().getValueAsDouble();
    double warmUp1RollerRPS = motor1RPS / Constants.Shooter.WarmUp1.MOTOR_ROTS_PER_ROLLER_ROTS;
    double shooterWheelRPS = warmUp1RollerRPS / Constants.Shooter.BeltRatios.SHOOTER_TO_WARMUP1;
    return shooterWheelRPSToSurfaceSpeed(shooterWheelRPS);
  }

  // Commands
  public Command ShootAtSpeed() {
    return Commands.runEnd(
        () -> this.setSpeed(Constants.Shooter.SHOOT_FOR_AUTO), 
        this::stop, 
        this
    );
  }

  @Override
  public void periodic() {
    DogLog.log("Shooter/TargetSurfaceSpeed", targetShooterSurfaceSpeed);
    DogLog.log("Shooter/IsAtSpeed", isAtSpeed());
    DogLog.log("Shooter/CurrentSurfaceSpeed", getCurrentSpeed());
    DogLog.log("Shooter/WarmUp1MotorRPS", warmUpMotor1.getVelocity().getValueAsDouble());
    DogLog.log("Shooter/WarmUp2MotorRPS", warmUpMotor2.getVelocity().getValueAsDouble());
    DogLog.log("Shooter/WarmUp3MotorRPS", warmUpMotor3.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // Update battery voltage for all motors
    double batteryVoltage = RobotController.getBatteryVoltage();
    warmUpMotor1SimState.setSupplyVoltage(batteryVoltage);
    warmUpMotor2SimState.setSupplyVoltage(batteryVoltage);
    warmUpMotor3SimState.setSupplyVoltage(batteryVoltage);

    // Get motor voltages from sim states
    double warmUp1Voltage = warmUpMotor1SimState.getMotorVoltage();
    double warmUp2Voltage = warmUpMotor2SimState.getMotorVoltage();
    double warmUp3Voltage = warmUpMotor3SimState.getMotorVoltage();

    // Update each flywheel simulation with its motor voltage
    warmUp1Sim.setInputVoltage(warmUp1Voltage);
    warmUp2Sim.setInputVoltage(warmUp2Voltage);
    warmUp3Sim.setInputVoltage(warmUp3Voltage);

    // Advance simulations
    warmUp1Sim.update(0.02); // 20ms period
    warmUp2Sim.update(0.02);
    warmUp3Sim.update(0.02);

    // Shooter wheel is mechanically coupled to warmUp1 through 18t:20t belt
    // Shooter angular velocity = warmUp1 roller velocity / belt_ratio
    double warmUp1RollerAngularVel = warmUp1Sim.getAngularVelocityRadPerSec();
    double shooterWheelAngularVel = warmUp1RollerAngularVel / Constants.Shooter.BeltRatios.SHOOTER_TO_WARMUP1;
    
    // Convert rad/s to rotations/s for TalonFX
    double warmUp1MotorVelocity = warmUp1Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double warmUp2MotorVelocity = warmUp2Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double warmUp3MotorVelocity = warmUp3Sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Set simulated velocities
    warmUpMotor1SimState.setRotorVelocity(warmUp1MotorVelocity);
    warmUpMotor2SimState.setRotorVelocity(warmUp2MotorVelocity);
    warmUpMotor3SimState.setRotorVelocity(warmUp3MotorVelocity);

    // Update positions (integrate velocity)
    warmUpMotor1SimState.addRotorPosition(warmUp1MotorVelocity * 0.02);
    warmUpMotor2SimState.addRotorPosition(warmUp2MotorVelocity * 0.02);
    warmUpMotor3SimState.addRotorPosition(warmUp3MotorVelocity * 0.02);

    // Log simulation data
    DogLog.log("Shooter/Sim/WarmUp1MotorRPS", warmUp1MotorVelocity);
    DogLog.log("Shooter/Sim/WarmUp2MotorRPS", warmUp2MotorVelocity);
    DogLog.log("Shooter/Sim/WarmUp3MotorRPS", warmUp3MotorVelocity);
    DogLog.log("Shooter/Sim/ShooterWheelAngularVelRadPerSec", shooterWheelAngularVel);
    DogLog.log("Shooter/Sim/BatteryVoltage", batteryVoltage);
  }
}