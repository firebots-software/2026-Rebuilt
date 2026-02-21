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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

  // Simulation Objects
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
            .withKV(Constants.Intake.Rollers.kV)
            .withKP(Constants.Intake.Rollers.kP)
            .withKI(Constants.Intake.Rollers.kI)
            .withKD(Constants.Intake.Rollers.kD);

    Slot0Configs armSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.Arm.kV)
            .withKP(Constants.Intake.Arm.kP)
            .withKI(Constants.Intake.Arm.kI)
            .withKD(Constants.Intake.Arm.kD)
            .withKG(Constants.Intake.Arm.kG);

    CurrentLimitsConfigs rollersCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT);

    CurrentLimitsConfigs armCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT);

    // Creates a FusedCANcoder, which combines data from the CANcoder and the arm
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

    // Add the CANcoder as a feedback source for the motor's built-in encoder
    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(cancoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT)
            .withRotorToSensorRatio(
                Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT
                    / Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT);

    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
    rollersConfig.Slot0 = rollersSlot0Configs;
    rollersConfig.CurrentLimits = rollersCurrentLimitsConfigs;
    rollersConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollersConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Slot0 = armSlot0Configs;
    armConfig.CurrentLimits = armCurrentLimitsConfigs;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.Feedback = feedbackConfigs;

    TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
    TalonFXConfigurator rollersMotorConfig = rollersMotor.getConfigurator();

    armMotorConfig.apply(armConfig);
    rollersMotorConfig.apply(rollersConfig);

    DogLog.log("Subsystems/Intake/Arm/Gains/kP", Constants.Intake.Arm.kP);
    DogLog.log("Subsystems/Intake/Arm/Gains/kI", Constants.Intake.Arm.kI);
    DogLog.log("Subsystems/Intake/Arm/Gains/kD", Constants.Intake.Arm.kD);
    DogLog.log("Subsystems/Intake/Arm/Gains/kV", Constants.Intake.Arm.kV);
    DogLog.log("Subsystems/Intake/Arm/Gains/kG", Constants.Intake.Arm.kG);

    DogLog.log("Subsystems/Intake/Rollers/Gains/kP", Constants.Intake.Rollers.kP);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kI", Constants.Intake.Rollers.kI);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kD", Constants.Intake.Rollers.kD);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kV", Constants.Intake.Rollers.kV);

    if (RobotBase.isSimulation()) setupSimulation();
  }

  private void setupSimulation() {
    rollersMotorSimState = rollersMotor.getSimState();
    armMotorSimState = armMotor.getSimState();
    armCancoderSimState = cancoder.getSimState();

    // Match real-motor sign conventions
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
                Constants.Intake.Rollers.Simulation.SIM_MOI_KG_M2,
                Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT),
            kraken60GearboxModel);

    armMechanismSim =
        new SingleJointedArmSim(
            kraken44GearboxModel,
            Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT,
            Constants.Intake.Arm.Simulation.SIM_MOI_KG_M2,
            Constants.Intake.Arm.Simulation.ARM_LENGTH_METERS,
            Units.degreesToRadians(Constants.Intake.Arm.Simulation.SIM_ARM_POS_MIN),
            Units.degreesToRadians(Constants.Intake.Arm.Simulation.SIM_ARM_POS_MAX),
            true,
            Units.degreesToRadians(Constants.Intake.Arm.ARM_POS_RETRACTED));
  }

  public void runRollers(double speedRollersRotationsPerSecond) {
    targetRollersRPS = speedRollersRotationsPerSecond;
    rollersMotor.setControl(
        m_velocityRequest.withVelocity(
            targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT));
  }

  public void stopRollers() {
    targetRollersRPS = 0;
    rollersMotor.setControl(m_velocityRequest.withVelocity(0));
  }

  public void setArmDegrees(double angleDeg) {
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_MAX);
    double targetArmRotations = targetAngleDeg / 360.0;
    armMotor.setControl(m_positionRequest.withPosition(targetArmRotations));
  }

  public void setPowerRetract() {
    armMotor.setControl(
        new TorqueCurrentFOC(Constants.Intake.Arm.POWER_REACT_TORQUE_CURRENT));
  }

  public Rotation2d getArmAbsolutePosition() {
    return new Rotation2d(
        Units.rotationsToRadians(
            getCancoderPositionRaw() * Constants.Intake.Arm.ARM_ROTS_PER_CANCODER_ROT));
  }

  public double getCancoderPositionRaw() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean atTargetSpeed() {
    return Math.abs(
            rollersMotor.getVelocity().getValueAsDouble()
                - targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT)
        <= Constants.Intake.Rollers.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  public Command runRollersUntilInterruptedCommand() {
    return startEnd(
        () -> this.runRollers(Constants.Intake.Rollers.TARGET_ROLLER_RPS), this::stopRollers);
  }

  public Command runRollersUntilInterruptedCommand(double targetRollers_RPS) {
    return startEnd(() -> this.runRollers(targetRollers_RPS), this::stopRollers);
  }

  public Command setArmToDegreesCommand(double degrees) {
    return runOnce(() -> this.setArmDegrees(degrees));
  }

  public Command intakeUntilInterruptedCommand() {
    return runEnd(
        () -> {
          setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
          runRollers(Constants.Intake.Rollers.TARGET_ROLLER_RPS);
        },
        this::stopRollers);
  }

  public Command powerRetractCommand() {
    return runOnce(this::setPowerRetract);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Intake/Rollers/CurrentSpeed (rps)", rollersMotor.getVelocity().getValueAsDouble() * Constants.Intake.Rollers.ROLLER_ROTS_PER_MOTOR_ROT);
    DogLog.log("Subsystems/Intake/Rollers/TargetSpeed (rps)", targetRollersRPS);
    DogLog.log("Subsystems/Intake/Rollers/AtTargetSpeed", atTargetSpeed());

    DogLog.log("Subsystems/Intake/Arm/AbsoluteEncoderRaw (rots)", getCancoderPositionRaw());
    DogLog.log(
        "Subsystems/Intake/Arm/CurrentPosition (degs)", getArmAbsolutePosition().getDegrees());
    DogLog.log(
        "Subsystems/Intake/Arm/TargetPosition (degs)", targetAngleDeg);
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

    // 1. How many volts applied to the motor?
    double batteryV = RobotController.getBatteryVoltage();
    rollersMotorSimState.setSupplyVoltage(batteryV);
    armMotorSimState.setSupplyVoltage(batteryV);

    double rollersAppliedVolts =
        rollersMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double armAppliedVolts =
        armMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    rollersMechanismSim.setInputVoltage(rollersAppliedVolts);
    armMechanismSim.setInputVoltage(armAppliedVolts);

    rollersMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    armMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 2. What happens to the simulated mechanism?
    double rollersMechPosRot = rollersMechanismSim.getAngularPositionRotations();
    double rollersMechVelRps = rollersMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double armMechAngleRad = armMechanismSim.getAngleRads();
    double armMechVelRps = armMechanismSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    double armMechPosRot = armMechAngleRad / (2.0 * Math.PI);

    // 3. Updating the simulated motor based on the behavior of the simulated mechanism
    double rollersRotorPosRot =
        rollersMechPosRot * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT;
    double rollersRotorVelRps =
        rollersMechVelRps * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT;

    double armRotorPosRot = armMechPosRot * Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT;
    double armRotorVelRps = armMechVelRps * Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT;

    rollersMotorSimState.setRawRotorPosition(rollersRotorPosRot);
    rollersMotorSimState.setRotorVelocity(rollersRotorVelRps);

    armMotorSimState.setRawRotorPosition(armRotorPosRot);
    armMotorSimState.setRotorVelocity(armRotorVelRps);

    // 4. Keep CANcoder sim in sync with arm mechanism position/velocity
    armCancoderSimState.setRawPosition(
        armMechPosRot * Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT);
    armCancoderSimState.setVelocity(
        armMechVelRps * Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT);

    // 4. What happens to the battery (simulated)?
    double rollersSupplyCurrentAmps = rollersMotorSimState.getSupplyCurrent();
    double armSupplyCurrentAmps = armMotorSimState.getSupplyCurrent();
    double totalSupplyCurrentAmps = rollersSupplyCurrentAmps + armSupplyCurrentAmps;
    double targetBatteryV = BatterySim.calculateDefaultBatteryLoadedVoltage(totalSupplyCurrentAmps);
    RoboRioSim.setVInVoltage(targetBatteryV);
  }
}
