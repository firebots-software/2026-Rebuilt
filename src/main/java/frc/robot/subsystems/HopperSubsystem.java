package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
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

public class HopperSubsystem extends SubsystemBase {
  private final LoggedTalonFX motor;
  private double targetSurfaceSpeed = 0;

  // Simulation objects
  private TalonFXSimState motorSimState;
  private DCMotorSim physicsSim;

  private static final double SIM_DT_SEC = 0.020; // time between updating the simulator
  private static final double HOPPER_MECHANISM_REDUCTION = 5.0; // motor:mechanism
  private static final double ESTIMATED_J_KG_M2 = 0.0012;

  public HopperSubsystem() {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.HOPPER_STATOR_LIMIT)
            .withSupplyCurrentLimit(Constants.Hopper.HOPPER_SUPPLY_LIMIT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Hopper.kP)
            .withKI(Constants.Hopper.kI)
            .withKD(Constants.Hopper.kD)
            .withKV(Constants.Hopper.kV);

    motor = new LoggedTalonFX(Constants.Hopper.MOTOR_PORT);

    MotorOutputConfigs moc =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    motor.getConfigurator().apply(s0c);
    motor.getConfigurator().apply(clc);
    motor.getConfigurator().apply(moc);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    motorSimState = motor.getSimState();
    motorSimState.Orientation = ChassisReference.Clockwise_Positive;
    motorSimState.setMotorType(
        TalonFXSimState.MotorType.KrakenX60); // set the type of motor being simulated

    var gearbox = DCMotor.getKrakenX60Foc(1);

    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, ESTIMATED_J_KG_M2, HOPPER_MECHANISM_REDUCTION),
            gearbox);
  }

  public void runHopper(double speed) {
    targetSurfaceSpeed = speed;
    motor.setControl(
        new VelocityVoltage(speed / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL));
  }

  public void stop() {
    targetSurfaceSpeed = 0;
    motor.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {
    return motor.getVelocity().getValueAsDouble()
            - targetSurfaceSpeed / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL
        <= Constants.Hopper.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  // Commands
  public Command RunHopper(double speed) {
    return Commands.runEnd(
        () -> this.runHopper(Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystem/Hopper/Speed", motor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystem/Hopper/AtSpeed", atSpeed());
  }

  @Override
  public void simulationPeriodic() {
    if (motorSimState == null || physicsSim == null) return;

    // 1) Supply voltage to CTRE sim
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2) Read applied motor voltage and step mechanism plant
    double appliedVolts =
        motorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    physicsSim.setInputVoltage(appliedVolts);
    physicsSim.update(SIM_DT_SEC);

    // 3) Mechanism-side sim -> rotor-side sensor state
    double mechPosRot = physicsSim.getAngularPositionRotations();
    double mechVelRps = physicsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double rotorPosRot = mechPosRot * HOPPER_MECHANISM_REDUCTION;
    double rotorVelRps = mechVelRps * HOPPER_MECHANISM_REDUCTION;

    motorSimState.setRawRotorPosition(rotorPosRot);
    motorSimState.setRotorVelocity(rotorVelRps);

    // 4) Battery sag model
    double loadedV =
        BatterySim.calculateDefaultBatteryLoadedVoltage(physicsSim.getCurrentDrawAmps());
    RoboRioSim.setVInVoltage(loadedV);

    DogLog.log("Subsystem/Hopper/Sim/AppliedVolts", appliedVolts);
    DogLog.log("Subsystem/Hopper/Sim/MechVelRps", mechVelRps);
    DogLog.log("Subsystem/Hopper/Sim/RotorVelRps", rotorVelRps);
    DogLog.log("Subsystem/Hopper/Sim/CurrentAmps", physicsSim.getCurrentDrawAmps());
    DogLog.log("Subsystem/Hopper/Sim/BatteryV", loadedV);
  }
}
