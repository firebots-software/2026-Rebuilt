package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.FuelGaugeDetection.GaugeCalculationType;
import frc.robot.util.LoggedTalonFX;

public class HopperSubsystem extends SubsystemBase {
  private final LoggedTalonFX hopperMotor;
  private double targetSurfaceSpeedMps = 0.0;

  // Simulation Objects
  private TalonFXSimState hopperMotorSimState;
  private DCMotorSim hopperMechanismSim;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  public HopperSubsystem() {
    CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.STATOR_LIMIT_AMPS)
            .withSupplyCurrentLimit(Constants.Hopper.SUPPLY_LIMIT_AMPS);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Hopper.kP)
            .withKI(Constants.Hopper.kI)
            .withKD(Constants.Hopper.kD)
            .withKV(Constants.Hopper.kV);

    hopperMotor = new LoggedTalonFX(Constants.Hopper.MOTOR_PORT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    TalonFXConfigurator hopperConfigurator = hopperMotor.getConfigurator();

    hopperConfigurator.apply(s0c);
    hopperConfigurator.apply(currentLimitConfigs);
    hopperConfigurator.apply(motorOutputConfigs);

    DogLog.log("Subsystems/Hopper/Gains/kP", Constants.Hopper.kP);
    DogLog.log("Subsystems/Hopper/Gains/kI", Constants.Hopper.kI);
    DogLog.log("Subsystems/Hopper/Gains/kD", Constants.Hopper.kD);
    DogLog.log("Subsystems/Hopper/Gains/kV", Constants.Hopper.kV);

    if (RobotBase.isSimulation()) setupSimulation();
  }

  private void setupSimulation() {
    hopperMotorSimState = hopperMotor.getSimState();
    hopperMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
    hopperMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    DCMotor krakenGearboxModel = DCMotor.getKrakenX60Foc(1);

    hopperMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel,
                Constants.Hopper.Simulation.MECHANISM_SIM_MOI_KG_M2,
                Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION),
            krakenGearboxModel);
  }

  public void runHopperMps(double targetSurfaceSpeedMps) {
    this.targetSurfaceSpeedMps = targetSurfaceSpeedMps;
    hopperMotor.setControl(
        m_velocityRequest.withVelocity(
            targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER));
  }

  public void stop() {
    runHopperMps(0.0);
  }

  public double getFloorSpeedMPS() {
    return hopperMotor.getVelocity().getValueAsDouble()
        * Constants.Hopper.BELT_TRAVEL_METERS_PER_MOTOR_ROTATION;
  }

  public double getAgitatorSpeedRPS() {
    return hopperMotor.getVelocity().getValueAsDouble()
        * Constants.Hopper.AGITATOR_ROTATIONS_PER_MOTOR_ROTATION;
  }

  public boolean atTargetSpeed() {
    return Math.abs(getFloorSpeedMPS() - targetSurfaceSpeedMps)
        <= Constants.Hopper.FLOOR_SPEED_TOLERANCE_MPS;
  }

  public boolean isHopperSufficientlyEmpty(FuelGaugeDetection fuelGaugeDetection) {
    return (fuelGaugeDetection != null
        ? fuelGaugeDetection.GaugeLessThanEqualTo(
            GaugeCalculationType.SMOOTHED_MULTIPLE_BALLS, FuelGauge.LOW)
        : false);
  }

  // Does not stop the Hopper when interrupted
  public Command runHopperCommand() {
    return runOnce(() -> runHopperMps(Constants.Hopper.TARGET_SURFACE_SPEED_MPS));
  }

  // Does not stop the Hopper when interrupted
  public Command runHopperCommand(double targetSurfaceSpeedMps) {
    return runOnce(() -> runHopperMps(targetSurfaceSpeedMps));
  }

  // Stops the Hopper when interrupted
  public Command runHopperUntilInterruptedCommand() {
    return startEnd(() -> runHopperMps(Constants.Hopper.TARGET_SURFACE_SPEED_MPS), this::stop);
  }

  // Stops the Hopper when interrupted
  public Command runHopperUntilInterruptedCommand(double targetSurfaceSpeedMps) {
    return startEnd(() -> runHopperMps(targetSurfaceSpeedMps), this::stop);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Hopper/CurrentSurfaceSpeed (mps)", hopperMotor.getVelocity().getValueAsDouble() * Constants.Hopper.BELT_TRAVEL_METERS_PER_MOTOR_ROTATION);
    DogLog.log("Subsystems/Hopper/TargetSurfaceSpeed (mps)", targetSurfaceSpeedMps);
    DogLog.log("Subsystems/Hopper/AtTargetSpeed", atTargetSpeed());
    DogLog.log(
        "Subsystems/Hopper/TargetMotorSpeed (rps)",
        targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER);
    DogLog.log(
        "Subsystems/Hopper/CurrentMotorSpeed (rps)", hopperMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    if (hopperMotorSimState == null || hopperMechanismSim == null) return;

    // 1. How many volts applied to the motor?
    hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedMotorVoltageVolts = hopperMotorSimState.getMotorVoltageMeasure().in(Units.Volts);
    hopperMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
    hopperMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 2. What happens to the simulated mechanism?
    double hopperMechanismVelocityRotationsPerSecond =
        hopperMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double hopperMechanismPositionRotations = hopperMechanismSim.getAngularPositionRotations();

    // 3. Updating the simulated motor based on the behavior of the simulated mechanism
    double motorRotorPositionRotations =
        hopperMechanismPositionRotations
            * Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION;
    double motorRotorVelocityRotationsPerSecond =
        hopperMechanismVelocityRotationsPerSecond
            * Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION;
    hopperMotorSimState.setRawRotorPosition(motorRotorPositionRotations);
    hopperMotorSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4. What happens to the battery (simulated)?
    double hopperSupplyCurrentAmps = hopperMotorSimState.getSupplyCurrent();
    double targetBatteryV =
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperSupplyCurrentAmps);
    RoboRioSim.setVInVoltage(targetBatteryV);
  }
}
