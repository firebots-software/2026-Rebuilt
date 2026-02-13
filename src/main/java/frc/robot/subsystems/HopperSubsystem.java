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
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    hopperMotor.getConfigurator().apply(s0c);
    hopperMotor.getConfigurator().apply(currentLimitConfigs);
    hopperMotor.getConfigurator().apply(motorOutputConfigs);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    hopperMotorSimState = hopperMotor.getSimState();
    hopperMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
    hopperMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var krakenGearboxModel = DCMotor.getKrakenX60Foc(1);

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
        new VelocityVoltage(
            targetSurfaceSpeedMetersPerSecond
                * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER));
  }

  public void stop() {
    targetSurfaceSpeedMetersPerSecond = 0.0;
    hopperMotor.setControl(new VelocityVoltage(targetSurfaceSpeedMetersPerSecond * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER));
  }

  public double getFloorSpeedMPS() {
    double measuredMotorSpeedRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
    return measuredMotorSpeedRotationsPerSecond
        * Constants.Hopper.HOPPER_BELT_METERS_PER_MOTOR_ROTATION;
  }

  public double getAgitatorSpeedRPS() {
    double measuredMotorSpeedRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
    return measuredMotorSpeedRotationsPerSecond
        * Constants.Hopper.AGITATOR_ROTATIONS_PER_MOTOR_ROTATION;
  }

  public boolean atTargetSpeed() {
    double measuredMotorSpeedRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
    double targetMotorSpeedRotationsPerSecond =
        targetSurfaceSpeedMetersPerSecond * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER;
    return Math.abs(measuredMotorSpeedRotationsPerSecond - targetMotorSpeedRotationsPerSecond)
        <= Constants.Hopper.HOPPER_VELOCITY_TOLERANCE_ROTATIONS_PER_SECOND;
  }

  // Commands
  public Command runHopperCommand() {
    return Commands.startEnd(
        () -> {
          targetSurfaceSpeedMetersPerSecond = Constants.Hopper.HOPPER_BELT_TARGET_SPEED_METERS_PER_SECOND;
          this.runHopper(targetSurfaceSpeedMetersPerSecond);
        }
          , this::stop, this);
  }

  // Commands
  public Command runHopperCommand(double speedMetersPerSec) {
    return Commands.startEnd(
        () -> {
          targetSurfaceSpeedMetersPerSecond = speedMetersPerSec;
          this.runHopper(targetSurfaceSpeedMetersPerSecond);
        }
          , this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Hopper/TargetMotorSpeed(RPS)", targetSurfaceSpeedMetersPerSecond * Constants.Hopper.MOTOR_ROTATIONS_PER_HOPPER_BELT_METER);
    DogLog.log("Subsystems/Hopper/CurrentMotorSpeed(RPS)", hopperMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Hopper/AppliedVolts", hopperMotor.getMotorVoltage().getValueAsDouble());
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
  }
}
