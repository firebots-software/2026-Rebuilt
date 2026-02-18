package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX armMotor, rollersMotor;
  private CANcoder cancoder;
  private double targetAngleDeg;
  private double targetRollersRPS;

  // Simulation objects
  private TalonFXSimState rollersMotorSimState;
  private TalonFXSimState armMotorSimState;
  private CANcoderSimState armCancoderSimState;
  private DCMotorSim rollersMechanismSim;
  private SingleJointedArmSim armMechanismSim;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);

  public IntakeSubsystem() {
    rollersMotor = new LoggedTalonFX(Constants.Intake.Rollers.CAN_ID);
    armMotor = new LoggedTalonFX(Constants.Intake.Arm.CAN_ID);
    targetAngleDeg = Constants.Intake.Arm.ARM_POS_RETRACTED;
    targetRollersRPS = 0;

    Slot0Configs rollersSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.Rollers.KV)
            .withKP(Constants.Intake.Rollers.KP)
            .withKI(Constants.Intake.Rollers.KI)
            .withKD(Constants.Intake.Rollers.KD);

    Slot0Configs armSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.Arm.KV)
            .withKP(Constants.Intake.Arm.KP)
            .withKI(Constants.Intake.Arm.KI)
            .withKD(Constants.Intake.Arm.KD)
            .withKG(Constants.Intake.Arm.KG);

    CurrentLimitsConfigs rollersCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT);

    CurrentLimitsConfigs armCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT);

    // creates a FusedCANcoder, which combines data from the CANcoder and the arm
    // motor's encoder
    cancoder = new CANcoder(Constants.Intake.Arm.ENCODER_PORT);
    CANcoderConfiguration ccConfig = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(Rotations.of(Constants.Intake.Arm.ENCODER_OFFSET));

    cancoder.getConfigurator().apply(ccConfig);
    cancoder.getConfigurator().apply(magnetSensorConfigs);

    // add the CANcoder as a feedback source for the motor's built-in encoder
    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(cancoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROTS)
            .withRotorToSensorRatio(
                Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROTS
                    / Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROTS);

    // Apply full TalonFXConfiguration objects to ensure factory defaults
    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
    rollersConfig.Slot0 = rollersSlot0Configs;
    rollersConfig.CurrentLimits = rollersCurrentLimitsConfigs;
    rollersConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Slot0 = armSlot0Configs;
    armConfig.CurrentLimits = armCurrentLimitsConfigs;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // changed side
    armConfig.Feedback = feedbackConfigs;

    TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
    TalonFXConfigurator rollersMotorConfig = rollersMotor.getConfigurator();

    armMotorConfig.apply(armConfig);
    rollersMotorConfig.apply(rollersConfig);

    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    rollersMotorSimState = rollersMotor.getSimState();
    armMotorSimState = armMotor.getSimState();
    armCancoderSimState = cancoder.getSimState();

    // Match your real-motor sign conventions
    rollersMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    armMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    rollersMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    armMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);

    var kraken60GearboxModel = DCMotor.getKrakenX60Foc(1);
    var kraken44GearboxModel = DCMotor.getKrakenX44Foc(1);

    rollersMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                kraken60GearboxModel,
                Constants.Intake.Rollers.SIM_MOI_KG_M2,
                Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS),
            kraken60GearboxModel);

    armMechanismSim =
        new SingleJointedArmSim(
            kraken44GearboxModel,
            Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROTS,
            Constants.Intake.Arm.SIM_MOI_KG_M2,
            Constants.Intake.Arm.ARM_LENGTH_METERS,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(Constants.Intake.Arm.SIM_ARM_POS_MAX),
            true,
            Units.degreesToRadians(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }

  public void runRollers(double speedRollersRotationsPerSecond) {
    targetRollersRPS = speedRollersRotationsPerSecond;
    rollersMotor.setControl(
        m_velocityRequest.withVelocity(
            targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS));
  }

  public void stopRollers() {
    targetRollersRPS = 0;
    rollersMotor.setControl(
        m_velocityRequest.withVelocity(
            targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS));
  }

  public void setArmDegrees(double angleDeg) {
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_MAX);
    double targetArmRotations = targetAngleDeg / 360.0;
    armMotor.setControl(m_positionRequest.withPosition(targetArmRotations));

    // double targetMotorRotations = targetAngleDeg /
    // Constants.Intake.Arm.ARM_DEGREES_PER_MOTOR_ROTS;

    // armMotor.setControl(m_positionRequest.withPosition(targetMotorRotations));
  }

  public Rotation2d getArmAbsolutePosition() {
    return new Rotation2d(
        (Units.rotationsToRadians(
            getCancoderPositionRaw() * Constants.Intake.Arm.ARM_ROTS_PER_CANCODER_ROTS)));
  }

  public double getCancoderPositionRaw() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean atTargetSpeed() {
    return Math.abs(
            rollersMotor.getVelocity().getValueAsDouble()
                - targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS)
        <= Constants.Intake.Rollers.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  public Command runRollersCommand() {
    return startEnd(
        () -> this.runRollers(Constants.Intake.Rollers.TARGET_ROLLER_RPS), this::stopRollers);
  }

  public Command runRollersCommand(double targetRollers_RPS) {
    return startEnd(() -> this.runRollers(targetRollers_RPS), this::stopRollers);
  }

  public Command armToDegrees(double degrees) {
    return runOnce(() -> this.setArmDegrees(degrees));
  }

  public Command extendArmAndRunRollers() {
    return runEnd(
        () -> {
          setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
          runRollers(Constants.Intake.Rollers.TARGET_ROLLER_RPS);
        },
        this::stopRollers);
  }

  @Override
  public void periodic() {
    // keep arm at current pos
    setArmDegrees(targetAngleDeg);
    DogLog.log(
        "Subsystems/Intake/Rollers/currentMotorRPS", rollersMotor.getVelocity().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake/Rollers/motorVolts", rollersMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake/Rollers/setpoint",
        rollersMotor.getClosedLoopReference().getValueAsDouble());
    DogLog.log("Subsystems/Intake/Arm/targetAngleDegs", targetAngleDeg);

    DogLog.log(
        "Subsystems/Intake/Arm/absoluteEncoderDegrees", getArmAbsolutePosition().getDegrees());
    DogLog.log("Subsystems/Intake/Arm/absoluteEncoderRaw", getCancoderPositionRaw());

    DogLog.log("Subsystems/Intake/Arm/currentMotorPos", armMotor.getPosition().getValueAsDouble());
    DogLog.log("Subsystems/Intake/Arm/motorVolts", armMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake/Arm/setpoint", armMotor.getClosedLoopReference().getValueAsDouble());
    // rollers
    // DogLog.log("Subsystems/Intake/Rollers/At target speed", atTargetSpeed());
    // DogLog.log("Subsystems/Intake/Rollers/Target Speed (rps)",
    // Constants.Intake.Rollers.TARGET_MOTOR_RPS);
    // DogLog.log(
    // "Subsystems/Intake/Rollers/Motor Velocity (rots/s)",
    // rollersMotor.getVelocity().getValueAsDouble());
    // DogLog.log(
    // "Subsystems/Intake/Rollers/Motor Position (rots)",
    // rollersMotor.getPosition().getValueAsDouble());
    // // arm
    // DogLog.log("Subsystems/Intake/Arm/CANcoder Position (raw)",
    // getCancoderPositionRaw());
    // DogLog.log(
    // "Subsystems/Intake/Arm/AbsolutePosition (degrees)",
    // getArmAbsolutePosition().getDegrees());
    // DogLog.log("Subsystems/Intake/Arm/TargetAngleDeg", targetAngleDeg);
  }

  @Override
  public void simulationPeriodic() {
    if (rollersMotorSimState == null
        || armMotorSimState == null
        || armCancoderSimState == null
        || rollersMechanismSim == null
        || armMechanismSim == null) {
      return;
    }

    // 1) Push current RoboRIO battery voltage into both Talon sims
    double batteryV = RobotController.getBatteryVoltage();
    rollersMotorSimState.setSupplyVoltage(batteryV);
    armMotorSimState.setSupplyVoltage(batteryV);

    // 2) Read controller output voltages and step both mechanism plants
    double rollersAppliedVolts =
        rollersMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double armAppliedVolts =
        armMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

    rollersMechanismSim.setInputVoltage(rollersAppliedVolts);
    armMechanismSim.setInputVoltage(armAppliedVolts);

    rollersMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    armMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 3) Mechanism-side state -> rotor-side state for Talon internal sensors
    double rollersMechPosRot = rollersMechanismSim.getAngularPositionRotations();
    double rollersMechVelRps = rollersMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double armMechAngleRad = armMechanismSim.getAngleRads();
    double armMechVelRps = armMechanismSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    double armMechPosRot = armMechAngleRad / (2.0 * Math.PI);

    double rollersRotorPosRot =
        rollersMechPosRot * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS;
    double rollersRotorVelRps =
        rollersMechVelRps * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROTS;

    double armRotorPosRot = armMechPosRot * Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROTS;
    double armRotorVelRps = armMechVelRps * Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROTS;

    rollersMotorSimState.setRawRotorPosition(rollersRotorPosRot);
    rollersMotorSimState.setRotorVelocity(rollersRotorVelRps);

    armMotorSimState.setRawRotorPosition(armRotorPosRot);
    armMotorSimState.setRotorVelocity(armRotorVelRps);

    // 4) Keep CANcoder sim in sync with arm mechanism position/velocity
    armCancoderSimState.setRawPosition(
        armMechPosRot * Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROTS);
    armCancoderSimState.setVelocity(
        armMechVelRps * Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROTS);

    // 5) Battery sag from total simulated current draw
    // double totalCurrentAmps = rollersMechanismSim.getCurrentDrawAmps() +
    // armMechanismSim.getCurrentDrawAmps();
    // double loadedBatteryV =
    // BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
    // RoboRioSim.setVInVoltage(loadedBatteryV);

    double rollersSupplyCurrentAmps = rollersMotorSimState.getSupplyCurrent();
    double armSupplyCurrentAmps = armMotorSimState.getSupplyCurrent();
    double totalSupplyCurrentAmps = rollersSupplyCurrentAmps + armSupplyCurrentAmps;

    // Calculate what the battery voltage should be with this load
    double targetBatteryV = BatterySim.calculateDefaultBatteryLoadedVoltage(totalSupplyCurrentAmps);

    RoboRioSim.setVInVoltage(targetBatteryV);
  }
}
