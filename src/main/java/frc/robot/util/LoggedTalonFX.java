package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import java.util.ArrayList;

public class LoggedTalonFX extends TalonFX {

  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
  private String name;
  private boolean refresh;
  private String temperature,
      closedLoopError,
      closedLoopReference,
      position,
      velocity,
      acceleration,
      supplycurrent,
      statorcurrent,
      torquecurrent,
      motorvoltage,
      supplyvoltage,
      error,
      reference,
      rotorPosition;

  public LoggedTalonFX(String deviceName, int deviceId, CANBus canbus) {
    super(deviceId, canbus);
    name = deviceName;
    init();
  }

  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId);
    name = deviceName;
    init();
  }

  public LoggedTalonFX(int deviceId, CANBus canbus) {
    super(deviceId, canbus);
    name = "motor " + deviceId;
    init();
  }

  public LoggedTalonFX(int deviceId) {
    super(deviceId);
    name = "motor " + deviceId;
    init();
  }

  public void init() {

    motors.add(this);
    this.getConfigurator().apply(new TalonFXConfiguration());
    this.temperature = name + "/temperature(degC)";
    this.closedLoopError = name + "/closedLoopError";
    this.closedLoopReference = name + "/closedLoopReference";
    this.position = name + "/position(rotations)";
    this.velocity = name + "/velocity(rps)";
    this.acceleration = name + "/acceleration(rps2)";
    this.supplycurrent = name + "/current/supply(A)";
    this.statorcurrent = name + "/current/stator(A)";
    this.torquecurrent = name + "/current/torque(A)";
    this.motorvoltage = name + "/voltage/motor(V)";
    this.supplyvoltage = name + "/voltage/supply(V)";
    this.error = name + "/closedloop/error";
    this.reference = name + "/closedloop/reference";
    this.rotorPosition = name + "/closedloop/rotorPosition";

    // Applying current limits
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40);
    // WITH A HIGH POWER MECHANISM, MAKE SURE TO INCREASE THE CURRENT LIMITS
    this.getConfigurator().apply(clc);
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

  // For some reason Robot.java doesn't recognize the static method here
  // when there is another method with the same name
  public static void periodic_static() {
    for (LoggedTalonFX l : motors) {
      l.periodic();
    }
  }

  public void periodic() {
    DogLog.log(temperature, this.getDeviceTemp().getValueAsDouble());
    DogLog.log(closedLoopError, this.getClosedLoopError().getValueAsDouble());
    DogLog.log(closedLoopReference, this.getClosedLoopReference().getValueAsDouble());

    DogLog.log(error, this.getClosedLoopError().getValueAsDouble());
    DogLog.log(reference, this.getClosedLoopReference().getValueAsDouble());
    DogLog.log(rotorPosition, this.getRotorPosition().getValueAsDouble());

    DogLog.log(position, this.getPosition().getValueAsDouble());
    DogLog.log(velocity, this.getVelocity().getValueAsDouble());
    DogLog.log(acceleration, this.getAcceleration().getValueAsDouble());

    // Current
    DogLog.log(supplycurrent, this.getSupplyCurrent().getValueAsDouble());
    DogLog.log(statorcurrent, this.getStatorCurrent().getValueAsDouble());
    DogLog.log(torquecurrent, this.getTorqueCurrent().getValueAsDouble());

    // Voltage
    DogLog.log(motorvoltage, this.getMotorVoltage().getValueAsDouble());
    DogLog.log(supplyvoltage, this.getSupplyVoltage().getValueAsDouble());
  }

  public static double getCurrentState() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentState'");
  }
}
