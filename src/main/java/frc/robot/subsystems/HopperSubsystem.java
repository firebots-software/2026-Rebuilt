package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.FuelGaugeDetection.GaugeCalculationType;
import frc.robot.util.LoggedTalonFX;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HopperSubsystem extends SubsystemBase {
  private final LoggedTalonFX hopperMotorMaster, hopperMotorSlave;
  private double targetSurfaceSpeedMps = 0.0;

  // private TalonFXSimState hopperMotorSimState;
  // private DCMotorSim hopperMechanismSim;

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

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs();
    clrc.withVoltageClosedLoopRampPeriod(0.3);

    hopperMotorMaster =
        new LoggedTalonFX("HopperFloorMaster", Constants.Hopper.MOTOR_PORT_MASTER, Constants.Swerve.CAN_BUS);
    hopperMotorSlave = new LoggedTalonFX("HopperFloorSlave", Constants.Hopper.MOTOR_PORT_SLAVE, Constants.Swerve.CAN_BUS);
    hopperMotorSlave.setControl(new Follower(hopperMotorMaster.getDeviceID(), MotorAlignmentValue.Aligned));

    TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
    hopperConfig.Slot0 = s0c;
    hopperConfig.CurrentLimits = currentLimitConfigs;
    hopperConfig.MotorOutput = motorOutputConfigs;
    hopperConfig.ClosedLoopRamps = clrc;

    TalonFXConfigurator hopperMotorMasterConfig = hopperMotorMaster.getConfigurator();
    TalonFXConfigurator hopperMotorSlaveConfig = hopperMotorSlave.getConfigurator();
    hopperMotorMasterConfig.apply(hopperConfig);
    hopperMotorSlaveConfig.apply(hopperConfig);

    DogLog.log("Subsystems/Hopper/Gains/kP", Constants.Hopper.kP);
    DogLog.log("Subsystems/Hopper/Gains/kI", Constants.Hopper.kI);
    DogLog.log("Subsystems/Hopper/Gains/kD", Constants.Hopper.kD);
    DogLog.log("Subsystems/Hopper/Gains/kV", Constants.Hopper.kV);

    // if (RobotBase.isSimulation()) setupSimulation();
  }

  // private void setupSimulation() {
  // hopperMotorSimState = hopperMotor.getSimState();
  // hopperMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
  // hopperMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

  // DCMotor krakenGearboxModel = DCMotor.getKrakenX60Foc(1);

  // hopperMechanismSim =
  // new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(
  // krakenGearboxModel,
  // Constants.Hopper.Simulation.MECHANISM_SIM_MOI_KG_M2,
  // Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION),
  // krakenGearboxModel);
  // }

  // Ruth's version
  public void runHopperMps(DoubleSupplier targetSurfaceSpeedMps, BooleanSupplier readyToRun) {
    if (readyToRun.getAsBoolean()) {
      this.targetSurfaceSpeedMps = targetSurfaceSpeedMps.getAsDouble();
      hopperMotorMaster.setControl(
          m_velocityRequest.withVelocity(
              this.targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER));
    } else {
      hopperMotorMaster.stopMotor();
    }
  }

  public void runHopperMps(double targetSurfaceSpeedMps) {
    this.targetSurfaceSpeedMps = targetSurfaceSpeedMps;
    hopperMotorMaster.setControl(
        m_velocityRequest.withVelocity(
            targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER));
  }

  public void stop() {
    hopperMotorMaster.stopMotor();
  }

  public double getFloorSpeedMPS() {
    return hopperMotorMaster.getCachedVelocityRps()
        * Constants.Hopper.BELT_TRAVEL_METERS_PER_MOTOR_ROTATION;
  }

  public double getAgitatorSpeedRPS() {
    return hopperMotorMaster.getCachedVelocityRps()
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

  public double getTargetHopperSpeed(double shooterSpeed) {
    return Constants.Hopper.HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.get(shooterSpeed);
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
  // Ruth's Version
  public Command runHopperUntilInterruptedCommand(
      DoubleSupplier targetSurfaceSpeedMps, BooleanSupplier readyToRun) {
    return runEnd(() -> runHopperMps(targetSurfaceSpeedMps, readyToRun), this::stop);
  }

  public Command runHopperUntilInterruptedCommand(double targetSurfaceSpeedMps) {
    return runEnd(() -> runHopperMps(targetSurfaceSpeedMps), this::stop);
  }

  public double grabHopperRecommendedSpeed(double speedOfShooter) {
    return Constants.Hopper.HOPPER_FPS_FOR_SHOOTER_WHEEL_RPS.get(speedOfShooter);
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Subsystems/Hopper/CurrentSurfaceSpeed (mps)",
        hopperMotorMaster.getCachedVelocityRps()
            * Constants.Hopper.BELT_TRAVEL_METERS_PER_MOTOR_ROTATION);
    DogLog.log("Subsystems/Hopper/TargetSurfaceSpeed (mps)", targetSurfaceSpeedMps);
    DogLog.log("Subsystems/Hopper/AtTargetSpeed", atTargetSpeed());
    DogLog.log(
        "Subsystems/Hopper/TargetMotorSpeed (rps)",
        targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTATIONS_PER_BELT_TRAVEL_METER);
    DogLog.log("Subsystems/Hopper/CurrentMotorSpeed (rps)", hopperMotorMaster.getCachedVelocityRps());
  }

  // @Override
  // public void simulationPeriodic() {
  // if (hopperMotorSimState == null || hopperMechanismSim == null) return;

  // // 1. How many volts applied to the motor?
  // hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

  // double appliedMotorVoltageVolts =
  // hopperMotorSimState.getMotorVoltageMeasure().in(Units.Volts);
  // hopperMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
  // hopperMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

  // // 2. What happens to the simulated mechanism?
  // double hopperMechanismVelocityRotationsPerSecond =
  // hopperMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
  // double hopperMechanismPositionRotations =
  // hopperMechanismSim.getAngularPositionRotations();

  // // 3. Updating the simulated motor based on the behavior of the simulated
  // mechanism
  // double motorRotorPositionRotations =
  // hopperMechanismPositionRotations
  // * Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION;
  // double motorRotorVelocityRotationsPerSecond =
  // hopperMechanismVelocityRotationsPerSecond
  // * Constants.Hopper.MOTOR_ROTATIONS_PER_FLOOR_PULLEY_ROTATION;
  // hopperMotorSimState.setRawRotorPosition(motorRotorPositionRotations);
  // hopperMotorSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

  // // 4. What happens to the battery (simulated)?
  // double hopperSupplyCurrentAmps = hopperMotorSimState.getSupplyCurrent();
  // double targetBatteryV =
  // BatterySim.calculateDefaultBatteryLoadedVoltage(hopperSupplyCurrentAmps);
  // RoboRioSim.setVInVoltage(targetBatteryV);
  // }
}
