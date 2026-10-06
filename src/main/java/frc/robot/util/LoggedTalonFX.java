package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.Constants;

import java.util.ArrayList;

/**
 * Team 3501 Version of TalonFX class that automatically sets Current Limits and logs motor
 * information.
 */
public class LoggedTalonFX extends TalonFX {

  /** List of all LoggedTalonFX motors on our robot. */
  private static final ArrayList<LoggedTalonFX> allMotors = new ArrayList<>();

  /** Name of motor instance. */
  private String name;

  private final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private String temperature,
      closedLoopError,
      closedLoopReference,
      position,
      velocity,
      acceleration,
      supplyCurrent,
      statorCurrent,
      torqueCurrent,
      motorVoltage,
      supplyVoltage,
      rotorPosition;

  private StatusSignal<?> deviceTempSignal;
  private StatusSignal<?> closedLoopErrorSignal;
  private StatusSignal<?> closedLoopReferenceSignal;
  private StatusSignal<?> rotorPositionSignal;
  private StatusSignal<?> positionSignal;
  private StatusSignal<?> velocitySignal;
  private StatusSignal<?> accelerationSignal;
  private StatusSignal<?> supplyCurrentSignal;
  private StatusSignal<?> statorCurrentSignal;
  private StatusSignal<?> torqueCurrentSignal;
  private StatusSignal<?> motorVoltageSignal;
  private StatusSignal<?> supplyVoltageSignal;

  private double deviceTempC;
  private double closedLoopErrorVal;
  private double closedLoopReferenceVal;
  private double rotorPositionVal;
  private double positionRotations;
  private double velocityRps;
  private double accelerationRps2;
  private double supplyCurrentA;
  private double statorCurrentA;
  private double torqueCurrentA;
  private double motorVoltageV;
  private double supplyVoltageV;

  private BaseStatusSignal[] cachedSignals;

  public double getCachedDeviceTempC() {
    return deviceTempC;
  }

  public double getCachedClosedLoopError() {
    return closedLoopErrorVal;
  }

  public double getCachedClosedLoopReference() {
    return closedLoopReferenceVal;
  }

  public double getCachedRotorPosition() {
    return rotorPositionVal;
  }

  public double getCachedPositionRotations() {
    return positionRotations;
  }

  public double getCachedVelocityRps() {
    return velocityRps;
  }

  public double getCachedAccelerationRps2() {
    return accelerationRps2;
  }

  public double getCachedSupplyCurrentA() {
    return supplyCurrentA;
  }

  public double getCachedStatorCurrentA() {
    return statorCurrentA;
  }

  public double getCachedTorqueCurrentA() {
    return torqueCurrentA;
  }

  public double getCachedMotorVoltageV() {
    return motorVoltageV;
  }

  public double getCachedSupplyVoltageV() {
    return supplyVoltageV;
  }

  private void cacheSignals() {
    deviceTempSignal = this.getDeviceTemp(false);
    closedLoopErrorSignal = this.getClosedLoopError(false);
    closedLoopReferenceSignal = this.getClosedLoopReference(false);
    rotorPositionSignal = this.getRotorPosition(false);
    positionSignal = this.getPosition(false);
    velocitySignal = this.getVelocity(false);
    accelerationSignal = this.getAcceleration(false);
    supplyCurrentSignal = this.getSupplyCurrent(false);
    statorCurrentSignal = this.getStatorCurrent(false);
    torqueCurrentSignal = this.getTorqueCurrent(false);
    motorVoltageSignal = this.getMotorVoltage(false);
    supplyVoltageSignal = this.getSupplyVoltage(false);

    cachedSignals =
        new BaseStatusSignal[] {
          deviceTempSignal,
          closedLoopErrorSignal,
          closedLoopReferenceSignal,
          rotorPositionSignal,
          positionSignal,
          velocitySignal,
          accelerationSignal,
          supplyCurrentSignal,
          statorCurrentSignal,
          torqueCurrentSignal,
          motorVoltageSignal,
          supplyVoltageSignal
        };

    // CRITICAL: Phoenix 6 defaults acceleration, torqueCurrent, and closedLoopError to 0 Hz.
    // We must explicitly set their update frequencies, or refreshAll() will timeout and freeze.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionSignal,
        velocitySignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        rotorPositionSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(
        20.0,
        accelerationSignal,
        torqueCurrentSignal,
        closedLoopErrorSignal,
        closedLoopReferenceSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(4.0, deviceTempSignal, supplyVoltageSignal);
  }

  public LoggedTalonFX(String deviceName, int deviceId, CANBus canbus) {
    super(deviceId, canbus);
    name = "Motors/" + deviceName;
    init();
  }

  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId, Constants.Swerve.CAN_BUS);
    name = "Motors/" + deviceName;
    init();
  }

  public LoggedTalonFX(int deviceId, CANBus canbus) {
    super(deviceId, canbus);
    name = "Motors/Motor " + deviceId;
    init();
  }

  public void init() {
    allMotors.add(this);

    this.temperature = name + "/temperature(degC)";
    this.closedLoopError = name + "/closedLoopError";
    this.closedLoopReference = name + "/closedLoopReference";
    this.position = name + "/position(rotations)";
    this.velocity = name + "/velocity(rps)";
    this.acceleration = name + "/acceleration(rps2)";
    this.supplyCurrent = name + "/current/supply(A)";
    this.statorCurrent = name + "/current/stator(A)";
    this.torqueCurrent = name + "/current/torque(A)";
    this.motorVoltage = name + "/voltage/motor(V)";
    this.supplyVoltage = name + "/voltage/supply(V)";
    this.rotorPosition = name + "/closedloop/rotorPosition";

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40);

    motorConfiguration.CurrentLimits = clc;
    this.getConfigurator().apply(motorConfiguration);
    cacheSignals();
  }

  public void updateCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit);

    this.getConfigurator().apply(clc);
  }

  public void updateCachedValues() {
    deviceTempC = deviceTempSignal.getValueAsDouble();
    closedLoopErrorVal = closedLoopErrorSignal.getValueAsDouble();
    closedLoopReferenceVal = closedLoopReferenceSignal.getValueAsDouble();
    rotorPositionVal = rotorPositionSignal.getValueAsDouble();
    positionRotations = positionSignal.getValueAsDouble();
    velocityRps = velocitySignal.getValueAsDouble();
    accelerationRps2 = accelerationSignal.getValueAsDouble();
    supplyCurrentA = supplyCurrentSignal.getValueAsDouble();
    statorCurrentA = statorCurrentSignal.getValueAsDouble();
    torqueCurrentA = torqueCurrentSignal.getValueAsDouble();
    motorVoltageV = motorVoltageSignal.getValueAsDouble();
    supplyVoltageV = supplyVoltageSignal.getValueAsDouble();
  }

  public void logValues() {
    DogLog.log(temperature, getCachedDeviceTempC());
    DogLog.log(closedLoopError, getCachedClosedLoopError());
    DogLog.log(closedLoopReference, getCachedClosedLoopReference());
    DogLog.log(rotorPosition, getCachedRotorPosition());
    DogLog.log(position, getCachedPositionRotations());
    DogLog.log(velocity, getCachedVelocityRps());
    DogLog.log(acceleration, getCachedAccelerationRps2());
    DogLog.log(supplyCurrent, getCachedSupplyCurrentA());
    DogLog.log(statorCurrent, getCachedStatorCurrentA());
    DogLog.log(torqueCurrent, getCachedTorqueCurrentA());
    DogLog.log(motorVoltage, getCachedMotorVoltageV());
    DogLog.log(supplyVoltage, getCachedSupplyVoltageV());
  }

  public static void periodic_static() {
    for (LoggedTalonFX motor : allMotors) {
      // Refresh per motor so an offline motor never blocks other motors
      BaseStatusSignal.refreshAll(motor.cachedSignals);
      motor.updateCachedValues();
      motor.logValues();
    }
  }
}