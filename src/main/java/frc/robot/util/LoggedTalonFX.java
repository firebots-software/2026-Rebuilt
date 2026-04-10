package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import java.util.ArrayList;

public class LoggedTalonFX extends TalonFX {

  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();
  private String name;
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
      supplyvoltage;

  public LoggedTalonFX(String deviceName, int deviceId, String canbus) {
    super(deviceId, canbus);
    name = deviceName;
    init();
  }

  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId);
    name = deviceName;
    init();
  }

  public LoggedTalonFX(int deviceId, String canbus) {
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
    this.temperature = "motors/" + name + "/temperature(degC)";
    this.closedLoopError = "motors/" + name + "/closedLoopError";
    this.closedLoopReference = "motors/" + name + "/closedLoopReference";
    this.position = "motors/" + name + "/position(rotations)";
    this.velocity = "motors/" + name + "/velocity(rps)";
    this.acceleration = "motors/" + name + "/acceleration(rps2)";
    this.supplycurrent = "motors/" + name + "/current/supply(A)";
    this.statorcurrent = "motors/" + name + "/current/stator(A)";
    this.torquecurrent = "motors/" + name + "/current/torque(A)";
    this.motorvoltage = "motors/" + name + "/voltage/motor(V)";
    this.supplyvoltage = "motors/" + name + "/voltage/supply(V)";
  }

  public static void peroidic() {
    for (LoggedTalonFX l : motors) {
      l.periodic();
    }
  }

  public void periodic() {
    DogLog.log(temperature, this.getDeviceTemp().getValueAsDouble());
    DogLog.log(closedLoopError, this.getClosedLoopError().getValueAsDouble());
    DogLog.log(closedLoopReference, this.getClosedLoopReference().getValueAsDouble());

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
}
