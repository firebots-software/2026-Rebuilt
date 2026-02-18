package frc.robot.util;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import java.util.ArrayList;

/**
 * Team 3501 Version of TalonFX class that automatically sets Current Limits and logs motor
 * information.
 */
public class LoggedTalonFX extends TalonFX {

  /** List of all LoggedTalonFX motors on our robot, defined. */
  private static ArrayList<LoggedTalonFX> motors = new ArrayList<>();

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
      error,
      reference,
      rotorPosition;

  /**
   * @param deviceName Designated name of this LoggedTalonFX
   * @param deviceId Motor ID of this LoggedTalonFX
   * @param canbus Name of CAN Bus Associated with this LoggedTalonFX. Might be deprecated to
   *     identify CAN Bus through string. Check phoenix6 documentation for more details.
   */
  public LoggedTalonFX(String deviceName, int deviceId, String canbus) {
    super(deviceId, canbus);
    name = deviceName;
    init();
  }

  /**
   * @param deviceName Designated name of this LoggedTalonFX
   * @param deviceId Motor ID of this LoggedTalonFX
   */
  public LoggedTalonFX(String deviceName, int deviceId) {
    super(deviceId);
    name = deviceName;
    init();
  }

  /**
   * @param deviceId Motor ID of this LoggedTalonFX
   * @param canbus Name of CAN Bus Associated with this LoggedTalonFX. Might be deprecated to
   *     identify CAN Bus through string. Check phoenix6 documentation for more details.
   */
  public LoggedTalonFX(int deviceId, String canbus) {
    super(deviceId, canbus);
    name = "motor " + deviceId;
    init();
  }

  /**
   * @param deviceId Motor ID of this LoggedTalonFX
   */
  public LoggedTalonFX(int deviceId) {
    super(deviceId);
    name = "motor " + deviceId;
    init();
  }

  /** Initializes strings that will be outputted through the LoggedTalonFX class. */
  public void init() {
    motors.add(this);
    this.getConfigurator().apply(new TalonFXConfiguration());
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

    motorConfiguration.CurrentLimits = clc;
    this.getConfigurator().apply(motorConfiguration);
  }

  public void updateCurrentLimits(double statorCurrentLimit, double supplyCurrentLimit) {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supplyCurrentLimit);

    motorConfiguration.CurrentLimits = clc;
    this.getConfigurator().apply(motorConfiguration);
  }

  // For some reason Robot.java doesn't recognize the static method here
  // when there is another method with the same name
  /**
   * This method will run the periodic() method for each of the motors and take care of the logging.
   * You must call LoggedTalonFX.periodic_static() in a periodic method in the code in order for
   * this to work.
   */
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
    DogLog.log(supplyCurrent, this.getSupplyCurrent().getValueAsDouble());
    DogLog.log(statorCurrent, this.getStatorCurrent().getValueAsDouble());
    DogLog.log(torqueCurrent, this.getTorqueCurrent().getValueAsDouble());

    // Voltage
    DogLog.log(motorVoltage, this.getMotorVoltage().getValueAsDouble());
    DogLog.log(supplyVoltage, this.getSupplyVoltage().getValueAsDouble());
  }
}
