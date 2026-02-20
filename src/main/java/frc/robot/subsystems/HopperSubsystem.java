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
  private double targetSurfaceSpeedMetersPerSecond = 0.0;

  // Simulation objects
  private TalonFXSimState hopperMotorSimState;
  private DCMotorSim hopperMechanismSim;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  public HopperSubsystem() {
    CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.HOPPER_STATOR_LIMIT_AMPS)
            .withSupplyCurrentLimit(Constants.Hopper.HOPPER_SUPPLY_LIMIT_AMPS);

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
                Constants.Hopper.HOPPER_SIM_MECHANISM_MOI_KG_M2,
                Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_PULLEY_ROTATION),
            krakenGearboxModel); // add stddevs later if it makes sense
  }

  public void runHopper(double targetSurfaceSpeedMetersPerSecond) {
    this.targetSurfaceSpeedMetersPerSecond = targetSurfaceSpeedMetersPerSecond;
    hopperMotor.setControl(
        m_velocityRequest.withVelocity(
            targetSurfaceSpeedMetersPerSecond
                * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER));
  }

  public void stop() {
    hopperMotor.setControl(m_velocityRequest.withVelocity(0));
  }

  public double getFloorSpeedMPS() {
    return hopperMotor.getVelocity().getValueAsDouble()
        * Constants.Hopper.HOPPER_BELT_METERS_PER_MOTOR_ROTATION;
  }

  public double getAgitatorSpeedRPS() {
    return hopperMotor.getVelocity().getValueAsDouble()
        * Constants.Hopper.AGITATOR_ROTATIONS_PER_MOTOR_ROTATION;
  }

  public boolean atTargetSpeed() {
    double measuredMotorSpeedRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
    double targetMotorSpeedRotationsPerSecond =
        targetSurfaceSpeedMetersPerSecond * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER;
    return Math.abs(measuredMotorSpeedRotationsPerSecond - targetMotorSpeedRotationsPerSecond)
        <= Constants.Hopper.HOPPER_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND;
  }

  public boolean isHopperSufficientlyEmpty(FuelGaugeDetection fuelGaugeDetection) {
    return (fuelGaugeDetection != null
        ? fuelGaugeDetection.GaugeLessThanEqualTo(
            GaugeCalculationType.SMOOTHED_MULTIPLE_BALLS, FuelGauge.LOW)
        : false);
  }

  // Commands
  public Command runHopperCommand() {
    return startEnd(
        () -> runHopper(Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND), this::stop);
  }

  // Commands
  public Command runHopperCommand(double speedMetersPerSec) {
    return startEnd(() -> runHopper(speedMetersPerSec), this::stop);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Hopper/Target Speed", targetSurfaceSpeedMetersPerSecond);
    DogLog.log("Subsystems/Hopper/At target speed", atTargetSpeed());
    DogLog.log(
        "Subsystems/Hopper/TargetMotorSpeed(RPS)",
        targetSurfaceSpeedMetersPerSecond * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER);
    DogLog.log(
        "Subsystems/Hopper/CurrentMotorSpeed(RPS)", hopperMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Hopper/AppliedVolts", hopperMotor.getMotorVoltage().getValueAsDouble());
    hopperMotor.getVelocity().getValueAsDouble();
    DogLog.log(
        "Subsystems/Hopper/Motor Current (stator)",
        hopperMotor.getStatorCurrent().getValueAsDouble());
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
    // DCMotorSim tracks the pulley/belt mechanism (after gear reduction)
    double hopperMechanismVelocityRotationsPerSecond =
        hopperMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double hopperMechanismPositionRotations = hopperMechanismSim.getAngularPositionRotations();

    // Convert mechanism rotations to motor rotor rotations
    // Motor spins faster: 5 motor rotations = 1 pulley rotation
    double motorRotorPositionRotations =
        hopperMechanismPositionRotations
            * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_PULLEY_ROTATION;
    double motorRotorVelocityRotationsPerSecond =
        hopperMechanismVelocityRotationsPerSecond
            * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_PULLEY_ROTATION;

    hopperMotorSimState.setRawRotorPosition(motorRotorPositionRotations);
    hopperMotorSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

    // 4) Battery sag model
    double loadedBatteryVoltageVolts =
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperMechanismSim.getCurrentDrawAmps());
    RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);

    double hopperSupplyCurrentAmps = hopperMotorSimState.getSupplyCurrent();

    // Calculate what the battery voltage should be with this load
    double targetBatteryV =
        BatterySim.calculateDefaultBatteryLoadedVoltage(hopperSupplyCurrentAmps);

    RoboRioSim.setVInVoltage(targetBatteryV);
  }
}
