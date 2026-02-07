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
  private final LoggedTalonFX hopperMotor;
  private double targetSurfaceSpeedMetersPerSecond = 0.0;

  // Simulation objects
  private TalonFXSimState hopperMotorSimState;
  private DCMotorSim hopperMechanismSim;

  public HopperSubsystem() {
    CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.HOPPER_STATOR_LIMIT)
            .withSupplyCurrentLimit(Constants.Hopper.HOPPER_SUPPLY_LIMIT);

    Slot0Configs velocityPidConfigs =
        new Slot0Configs()
            .withKP(Constants.Hopper.kP)
            .withKI(Constants.Hopper.kI)
            .withKD(Constants.Hopper.kD)
            .withKV(Constants.Hopper.kV);

    hopperMotor = new LoggedTalonFX(Constants.Hopper.MOTOR_PORT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    hopperMotor.getConfigurator().apply(velocityPidConfigs);
    hopperMotor.getConfigurator().apply(currentLimitConfigs);
    hopperMotor.getConfigurator().apply(motorOutputConfigs);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    hopperMotorSimState = hopperMotor.getSimState();
    hopperMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
    hopperMotorSimState.setMotorType(
        TalonFXSimState.MotorType.KrakenX60); // set the type of motor being simulated

    var krakenGearboxModel = DCMotor.getKrakenX60Foc(1);

    hopperMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel,
                Constants.Hopper.ESTIMATED_HOPPER_MOI_KG_M2,
                1.0 / Constants.Hopper.MOTOR_ROTS_TO_PULLEY_ROTS),
            krakenGearboxModel);
  }

  public void runHopper(double targetSurfaceSpeedMetersPerSecond) {
    this.targetSurfaceSpeedMetersPerSecond = targetSurfaceSpeedMetersPerSecond;
    hopperMotor.setControl(
        new VelocityVoltage(
            targetSurfaceSpeedMetersPerSecond
                / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL));
  }

  public void stop() {
    targetSurfaceSpeedMetersPerSecond = 0.0;
    hopperMotor.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {
    double measuredMotorSpeedRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
    double targetMotorSpeedRotationsPerSecond =
        targetSurfaceSpeedMetersPerSecond
            / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL;
    return Math.abs(measuredMotorSpeedRotationsPerSecond - targetMotorSpeedRotationsPerSecond)
        <= Constants.Hopper.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  // placeholder boolean function for seeing how many balls are in hopper
  public boolean isHopperSufficientlyEmpty() {
    return true;
  }

  // Commands
  public Command runHopperCommand() {
    return Commands.runEnd(
        () -> this.runHopper(targetSurfaceSpeedMetersPerSecond), this::stop, this);
  }

  public Command runHopperCommand(double speedMetersPerSec) {
    return Commands.runEnd(() -> this.runHopper(speedMetersPerSec), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Subsystems/Hopper/MotorSpeedRotationsPerSecond",
        hopperMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/sHopper/AtSpeed", atSpeed());
  }

  @Override
  public void simulationPeriodic() {
    if (hopperMotorSimState == null || hopperMechanismSim == null) return;

    // 1) Supply voltage to CTRE sim
    hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // 2) Read applied motor voltage and step mechanism plant
    double appliedMotorVoltageVolts =
        hopperMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    hopperMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
    hopperMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 3) Mechanism-side sim -> rotor-side sensor state
    double hopperMechanismPositionRotations = hopperMechanismSim.getAngularPositionRotations();
    double hopperMechanismVelocityRotationsPerSecond =
        hopperMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double motorRotorPositionRotations =
        hopperMechanismPositionRotations * 1.0 / Constants.Hopper.MOTOR_ROTS_TO_PULLEY_ROTS;
    double motorRotorVelocityRotationsPerSecond =
        hopperMechanismVelocityRotationsPerSecond
            * 1.0
            / Constants.Hopper.MOTOR_ROTS_TO_PULLEY_ROTS;

    hopperMotorSimState.setRawRotorPosition(motorRotorPositionRotations);
    hopperMotorSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4) Battery sag model
    double loadedBatteryVoltageVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperMechanismSim.getCurrentDrawAmps());
    RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);

    DogLog.log("Subsystems/Hopper/Sim/AppliedMotorVoltageVolts", appliedMotorVoltageVolts);
    DogLog.log(
        "Subsystems/Hopper/Sim/HopperMechanismVelocityRotationsPerSecond",
        hopperMechanismVelocityRotationsPerSecond);
    DogLog.log(
        "Subsystems/Hopper/Sim/MotorRotorVelocityRotationsPerSecond",
        motorRotorVelocityRotationsPerSecond);
    DogLog.log("Subsystems/Hopper/Sim/MotorCurrentAmps", hopperMechanismSim.getCurrentDrawAmps());
    DogLog.log("Subsystems/Hopper/Sim/LoadedBatteryVoltageVolts", loadedBatteryVoltageVolts);
  }
}
