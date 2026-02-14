package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX armMotor, intakeMotor;
  private CANcoder cancoder;
  private double targetAngleDeg;

  // Simulation objects
  private TalonFXSimState intakeMotorSimState;
  private TalonFXSimState armMotorSimState;
  private CANcoderSimState armCancoderSimState;
  private DCMotorSim intakeMechanismSim;
  private SingleJointedArmSim armMechanismSim;

  // 20 ms main loop
  private static final double SIM_DT_SEC = 0.020;

  // Intake roller sim assumptions
  private static final double INTAKE_SIM_MOI_KG_M2 = 0.0025;
  private static final double INTAKE_MECH_RATIO = 1.0 / Constants.Intake.MOTOR_ROTS_TO_INTAKE_ROTS;

  // Arm sim assumptions (77.8:1 from constants)
  private static final double ARM_SIM_MOI_KG_M2 = 0.02;
  private static final double ARM_MECH_RATIO = 1.0 / Constants.Intake.Arm.MOTOR_ROTS_TO_ARM_ROTS;
  private static final double ARM_LENGTH_METERS = 0.35;
  private static final double ARM_MIN_ANGLE_RAD = Math.toRadians(0.0);
  private static final double ARM_MAX_ANGLE_RAD =
      Math.toRadians(Constants.Intake.Arm.ARM_DEGREES_UPPER_LIMIT);

  public IntakeSubsystem() {
    intakeMotor = new LoggedTalonFX(Constants.Intake.INTAKE_MOTOR.port);
    armMotor = new LoggedTalonFX(Constants.Intake.Arm.ARM_MOTOR.port);
    targetAngleDeg = Constants.Intake.Arm.ARM_POS_INITIAL;

    Slot0Configs intakeSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.INTAKE_KV)
            .withKP(Constants.Intake.INTAKE_KP)
            .withKI(Constants.Intake.INTAKE_KI)
            .withKD(Constants.Intake.INTAKE_KD);

    Slot0Configs armSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.Arm.ARM_KV)
            .withKP(Constants.Intake.Arm.ARM_KP)
            .withKI(Constants.Intake.Arm.ARM_KI)
            .withKD(Constants.Intake.Arm.ARM_KD);

    CurrentLimitsConfigs intakeCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Intake.INTAKE_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.INTAKE_SUPPLY_CURRENT_LIMIT);

    CurrentLimitsConfigs armCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Intake.Arm.ARM_STATOR_CURRENT_LIMIT);

    TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
    TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();

    armMotorConfig.apply(armSlot0Configs);
    armMotorConfig.apply(armCurrentLimitsConfigs);
    armMotorConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    intakeMotorConfig.apply(intakeSlot0Configs);
    intakeMotorConfig.apply(intakeCurrentLimitsConfigs);
    intakeMotorConfig.apply(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    // creates a FusedCANcoder, which combines data from the CANcoder and the arm
    // motor's encoder
    cancoder = new CANcoder(Constants.Intake.Arm.ENCODER_PORT);
    CANcoderConfiguration ccConfig = new CANcoderConfiguration();
    // zero the magnet
    ccConfig
        .MagnetSensor
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(Rotations.of(Constants.Intake.Arm.ENCODER_OFFSET));
    cancoder.getConfigurator().apply(ccConfig);

    // add the CANcoder as a feedback source for the motor's built-in encoder
    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    fxConfig
        .Feedback
        .withFeedbackRemoteSensorID(cancoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(Constants.Intake.Arm.ENCODER_ROTS_TO_ARM_ROTS)
        .withRotorToSensorRatio(Constants.Intake.Arm.ARM_DEGREES_TO_MOTOR_ROTS);
    armMotorConfig.apply(fxConfig);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    intakeMotorSimState = intakeMotor.getSimState();
    armMotorSimState = armMotor.getSimState();
    armCancoderSimState = cancoder.getSimState();

    // Match your real-motor sign conventions
    intakeMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    armMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    intakeMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    armMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    var krakenGearboxModel = DCMotor.getKrakenX60Foc(1);

    intakeMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                krakenGearboxModel, INTAKE_SIM_MOI_KG_M2, INTAKE_MECH_RATIO),
            krakenGearboxModel);

    armMechanismSim =
        new SingleJointedArmSim(
            krakenGearboxModel,
            ARM_MECH_RATIO,
            ARM_SIM_MOI_KG_M2,
            ARM_LENGTH_METERS,
            ARM_MIN_ANGLE_RAD,
            ARM_MAX_ANGLE_RAD,
            true,
            ARM_MIN_ANGLE_RAD);
  }

  public void run(double speed) {
    intakeMotor.setControl(
        new VelocityVoltage(speed * Constants.Intake.MOTOR_ROTS_TO_INTAKE_ROTS)
            .withFeedForward(Constants.Intake.INTAKE_FEEDFORWARD));
  }

  public void stop() {
    intakeMotor.setControl(new VelocityVoltage(0));
  }

  public void setArmDegrees(double angleDeg) {
    targetAngleDeg = MathUtil.clamp(angleDeg, 0.0, Constants.Intake.Arm.ARM_DEGREES_UPPER_LIMIT);

    // Convert arm-degrees target to motor rotations for the request
    double targetMotorRotations = targetAngleDeg * Constants.Intake.Arm.ARM_DEGREES_TO_MOTOR_ROTS;

    armMotor.setControl(
        new PositionTorqueCurrentFOC(targetMotorRotations)
            .withFeedForward(Constants.Intake.Arm.ARM_FEEDFORWARD));
  }

  public double getCancoderPosition() {
    // uses the cancoder's position data directly
    return cancoder.getAbsolutePosition().getValueAsDouble()
        / Constants.Intake.ENCODER_ROTS_TO_INTAKE_ROTS;
  }

  public double getCancoderPositionRaw() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderPosition() {
    return armMotor.getPosition().getValueAsDouble() / Constants.Intake.ENCODER_ROTS_TO_INTAKE_ROTS;
  }

  public double getEncoderPositionRaw() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public boolean atSpeed() {
    return Math.abs(
            intakeMotor.getVelocity().getValueAsDouble() - Constants.Intake.INTAKE_TARGET_SPEED)
        <= Constants.Intake.Arm.ARM_TOLERANCE_DEGREES;
  }

  // Commands
  public Command runIntake() {
    return Commands.runEnd(() -> this.run(Constants.Intake.INTAKE_TARGET_SPEED), this::stop, this);
  }

  public Command armToDegrees(double degrees) {
    return Commands.runOnce(() -> this.setArmDegrees(degrees), this);
  }

  @Override
  public void periodic() {
    // rollers
    DogLog.log("Subsystems/Intake/Rollers/Target Speed", Constants.Intake.INTAKE_TARGET_SPEED);
    DogLog.log("Subsystems/Intake/Rollers/At target speed", atSpeed());
    DogLog.log(
        "Subsystems/Intake/Rollers/Motor Velocity (rots per sec)",
        intakeMotor.getVelocity().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake/Rollers/Motor Velocity (ft per sec)",
        intakeMotor.getVelocity().getValueAsDouble()
            * Constants.Intake.INTAKE_ROTS_PER_SEC_TO_FEET_PER_SEC);
    DogLog.log(
        "Subsystems/Intake/Rollers/Motor Position (rots)",
        intakeMotor.getPosition().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake/Rollers/Motor Current (stator)",
        intakeMotor.getStatorCurrent().getValueAsDouble());

    // arm
    DogLog.log("Subsystems/Intake/Arm/CANcoder Position (degrees)", getCancoderPosition());
    DogLog.log("Subsystems/Intake/Arm/CANcoder Position (raw)", getCancoderPositionRaw());
    DogLog.log("Subsystems/Intake/Arm/Position (degrees)", getEncoderPosition());
    DogLog.log("Subsystems/Intake/Arm/Position (raw)", getEncoderPositionRaw());
    DogLog.log(
        "Subsystems/Intake-Arm/Motor Velocity (rots/s)", armMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Intake-Arm/TargetAngleDeg", targetAngleDeg);
  }

  @Override
  public void simulationPeriodic() {
    if (intakeMotorSimState == null
        || armMotorSimState == null
        || armCancoderSimState == null
        || intakeMechanismSim == null
        || armMechanismSim == null) {
      return;
    }

    // 1) Push current RoboRIO battery voltage into both Talon sims
    double batteryV = RobotController.getBatteryVoltage();
    intakeMotorSimState.setSupplyVoltage(batteryV);
    armMotorSimState.setSupplyVoltage(batteryV);

    // 2) Read controller output voltages and step both mechanism plants
    double intakeAppliedVolts =
        intakeMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double armAppliedVolts =
        armMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

    intakeMechanismSim.setInputVoltage(intakeAppliedVolts);
    armMechanismSim.setInputVoltage(armAppliedVolts);

    intakeMechanismSim.update(SIM_DT_SEC);
    armMechanismSim.update(SIM_DT_SEC);

    // 3) Mechanism-side state -> rotor-side state for Talon internal sensors
    double intakeMechPosRot = intakeMechanismSim.getAngularPositionRotations();
    double intakeMechVelRps = intakeMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double armMechAngleRad = armMechanismSim.getAngleRads();
    double armMechVelRps = armMechanismSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    double armMechPosRot = armMechAngleRad / (2.0 * Math.PI);

    double intakeRotorPosRot = intakeMechPosRot * INTAKE_MECH_RATIO;
    double intakeRotorVelRps = intakeMechVelRps * INTAKE_MECH_RATIO;

    double armRotorPosRot = armMechPosRot * ARM_MECH_RATIO;
    double armRotorVelRps = armMechVelRps * ARM_MECH_RATIO;

    intakeMotorSimState.setRawRotorPosition(intakeRotorPosRot);
    intakeMotorSimState.setRotorVelocity(intakeRotorVelRps);

    armMotorSimState.setRawRotorPosition(armRotorPosRot);
    armMotorSimState.setRotorVelocity(armRotorVelRps);

    // 4) Keep CANcoder sim in sync with arm mechanism position/velocity
    armCancoderSimState.setRawPosition(armMechPosRot);
    armCancoderSimState.setVelocity(armMechVelRps);

    // 5) Battery sag from total simulated current draw
    double totalCurrentAmps =
        intakeMechanismSim.getCurrentDrawAmps() + armMechanismSim.getCurrentDrawAmps();
    double loadedBatteryV = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
    RoboRioSim.setVInVoltage(loadedBatteryV);

    DogLog.log("Subsystems/Intake-Sim/IntakeAppliedVolts", intakeAppliedVolts);
    DogLog.log("Subsystems/Intake-Sim/ArmAppliedVolts", armAppliedVolts);
    DogLog.log("Subsystems/Intake-Sim/IntakeMechVelRps", intakeMechVelRps);
    DogLog.log("Subsystems/Intake-Sim/ArmMechVelRps", armMechVelRps);
    DogLog.log("Subsystems/Intake-Sim/IntakeRotorVelRps", intakeRotorVelRps);
    DogLog.log("Subsystems/Intake-Sim/ArmRotorVelRps", armRotorVelRps);
    DogLog.log("Subsystems/Intake-Sim/TotalCurrentAmps", totalCurrentAmps);
    DogLog.log("Subsystems/Intake-Sim/LoadedBatteryVolts", loadedBatteryV);
    DogLog.log("Subsystems/Intake-Sim/ArmMechAngleDeg", Math.toDegrees(armMechAngleRad));
  }
}
