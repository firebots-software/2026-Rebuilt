package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX armMotor, rollersMotor;
  private CANcoder cancoder;

  private double targetAngleDeg;
  private double targetRollersRPS;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final TorqueCurrentFOC m_torqueCurrentRequest = new TorqueCurrentFOC(0);

  public IntakeSubsystem() {
    rollersMotor = new LoggedTalonFX("IntakeRollers", Constants.Intake.Rollers.CAN_ID);
    armMotor =
        new LoggedTalonFX("IntakeArm", Constants.Intake.Arm.CAN_ID, Constants.Swerve.CAN_BUS);
    targetAngleDeg = 0;
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
            .withKG(Constants.Intake.Arm.kG)
            .withGravityArmPositionOffset(Constants.Intake.Arm.GRAVITY_POS_OFFSET)
            .withKS(Constants.Intake.Arm.kS)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    CurrentLimitsConfigs rollersCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Rollers.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.Rollers.SUPPLY_CURRENT_LIMIT);

    CurrentLimitsConfigs armCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Intake.Arm.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Intake.Arm.SUPPLY_CURRENT_LIMIT);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Intake.Arm.mmcV)
            .withMotionMagicAcceleration(Constants.Intake.Arm.mmcA);

    ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1);

    // Creates a FusedCANcoder, which combines data from the CANcoder and the arm
    // motor's encoder
    cancoder = new CANcoder(Constants.Intake.Arm.ENCODER_PORT, Constants.Swerve.CAN_BUS);
    CANcoderConfiguration ccConfig = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withMagnetOffset(Rotations.of(Constants.Intake.Arm.ENCODER_OFFSET));

    cancoder.getConfigurator().apply(ccConfig);
    cancoder.getConfigurator().apply(magnetSensorConfigs);

    // Add the CANcoder as a feedback source for the motor's built-in encoder
    FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(cancoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(
                Constants.Intake.Arm.MOTOR_ROTS_PER_ARM_ROT
                    / Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT)
            .withSensorToMechanismRatio(Constants.Intake.Arm.CANCODER_ROTS_PER_ARM_ROT);

    TalonFXConfiguration rollersConfig =
        new TalonFXConfiguration()
            .withSlot0(rollersSlot0Configs)
            .withCurrentLimits(rollersCurrentLimitsConfigs)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withClosedLoopRamps(clrc);

    TalonFXConfiguration armConfig =
        new TalonFXConfiguration()
            .withSlot0(armSlot0Configs)
            .withMotionMagic(mmc)
            .withCurrentLimits(armCurrentLimitsConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

    TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
    TalonFXConfigurator rollersMotorConfig = rollersMotor.getConfigurator();

    armMotorConfig.apply(armConfig);
    rollersMotorConfig.apply(rollersConfig);

    DogLog.log("Subsystems/Intake/Arm/Gains/kP", Constants.Intake.Arm.kP);
    DogLog.log("Subsystems/Intake/Arm/Gains/kI", Constants.Intake.Arm.kI);
    DogLog.log("Subsystems/Intake/Arm/Gains/kD", Constants.Intake.Arm.kD);
    DogLog.log("Subsystems/Intake/Arm/Gains/kV", Constants.Intake.Arm.kV);
    DogLog.log("Subsystems/Intake/Arm/Gains/mmcV", Constants.Intake.Arm.mmcV);
    DogLog.log("Subsystems/Intake/Arm/Gains/mmcA", Constants.Intake.Arm.mmcA);

    DogLog.log("Subsystems/Intake/Rollers/Gains/kP", Constants.Intake.Rollers.kP);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kI", Constants.Intake.Rollers.kI);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kD", Constants.Intake.Rollers.kD);
    DogLog.log("Subsystems/Intake/Rollers/Gains/kV", Constants.Intake.Rollers.kV);
  }

  public void runRollers(double output) {
    rollersMotor.setControl(m_dutyCycleRequest.withOutput(output));
  }

  public void runRollers() {
    runRollers(1);
  }

  public void runRollersInReverse() {
    runRollers(-1);
  }

  public void runRollersWithVelocity(double speedRollersRotationsPerSecond) {
    targetRollersRPS = speedRollersRotationsPerSecond;
    rollersMotor.setControl(
        m_velocityRequest.withVelocity(
            targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT));
  }

  public void stopRollers() {
    targetRollersRPS = 0;
    rollersMotor.setControl(m_dutyCycleRequest.withOutput(0));
  }

  public Command stopRollersCommand() {
    return runOnce(this::stopRollers);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void setArmDegrees(double angleDeg) {
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, Constants.Intake.Arm.ARM_POS_MIN, Constants.Intake.Arm.ARM_POS_RETRACTED);
    double targetArmRotations = targetAngleDeg / 360.0;
    armMotor.setControl(m_motionMagicRequest.withPosition(targetArmRotations));
  }

  public void setPowerRetract() {
    setTorqueCurrent(Constants.Intake.Arm.POWER_RETRACT_TORQUE_CURRENT);
  }

  public void setTorqueCurrent(double current) {
    armMotor.setControl(m_torqueCurrentRequest.withOutput(current));
  }

  public void setPowerRetractPosition() {
    double targetRotations = Constants.Intake.Arm.ARM_POS_RETRACTED / 360.0;
    armMotor.setControl(m_positionRequest.withPosition(targetRotations));
  }

  public Rotation2d getArmUnfusedPosition() {
    return new Rotation2d(
        Units.rotationsToRadians(
            getCancoderPositionRaw() * Constants.Intake.Arm.ARM_ROTS_PER_CANCODER_ROT));
  }

  public Rotation2d getArmPosition() {
    return new Rotation2d(Units.rotationsToRadians(armMotor.getCachedPositionRotations()));
  }

  public void applyCoastConfigArm() {
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void applyBrakeConfigArm() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getCancoderPositionRaw() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean atTargetAngle() {
    return Math.abs(getArmPosition().getDegrees() - targetAngleDeg)
        <= Constants.Intake.Arm.POSITION_TOLERANCE_DEGREES;
  }

  public boolean atExtendedPosition() {
    return Math.abs(getArmPosition().getDegrees() - Constants.Intake.Arm.ARM_POS_EXTENDED)
        <= Constants.Intake.Arm.POSITION_TOLERANCE_DEGREES;
  }

  public boolean atTargetSpeed() {
    return Math.abs(
            rollersMotor.getCachedVelocityRps()
                - targetRollersRPS * Constants.Intake.Rollers.MOTOR_ROTS_PER_ROLLERS_ROT)
        <= Constants.Intake.Rollers.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  public Command runRollersUntilInterruptedCommand() {
    return startEnd(this::runRollers, this::stopRollers);
  }

  public Command runRollersUntilInterruptedCommand(double targetRollers_RPS) {
    return startEnd(this::runRollers, this::stopRollers);
  }

  public Command setArmToDegreesCommand(double degrees) {
    return runOnce(() -> setArmDegrees(degrees));
  }

  public Command retractIntakeCommand() {
    return run(
        () -> {
          setArmDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED);
          stopRollers();
        });
  }

  public Command powerRetractRollersCommand() {
    return runOnce(
        () -> {
          setPowerRetract();
          runRollers();
        });
    // .beforeStarting(Commands.waitSeconds(Constants.Intake.Arm.POWER_RETRACT_DELAY));
  }

  public Command torqueRetractCommand() {
    return runOnce(this::setPowerRetract);
  }

  public Command intakeUntilInterruptedCommand() {
    return runEnd(
        () -> {
          setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
          runRollers();
        },
        this::stopRollers);
  }

  public Command outtakeUntilInterruptedCommand() {
    return runEnd(
        () -> {
          setArmDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED);
          runRollersInReverse();
        },
        this::stopRollers);
  }

  public Command intakeDefault() {
    return runOnce(
        () -> {
          runRollersWithVelocity(Constants.Intake.Rollers.IDLE_ROLLER_VELO_RPS);
          setArmDegrees(Constants.Intake.Arm.ARM_POS_IDLE);
        });
  }

  public Command agitateArmCommand() {
    // stole these numbers from 5507, subject to change
    return Commands.sequence(
        run(this::setPowerRetract).withTimeout(4.5),
        Commands.sequence(
                runOnce(this::setPowerRetract).withTimeout(0.04),
                Commands.waitSeconds(0.3),
                setArmToDegreesCommand(Constants.Intake.Arm.ARM_POS_IDLE).withTimeout(0.04),
                Commands.waitSeconds(0.3))
            .repeatedly());
  }

  public Command powerRetractThenAgitateArmCommand() {
    return Commands.sequence(
        powerRetractRollersCommand(), Commands.waitSeconds(1), agitateArmCommand());
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Subsystems/Intake/Rollers/CurrentSpeed (rps)",
        rollersMotor.getCachedVelocityRps() * Constants.Intake.Rollers.ROLLER_ROTS_PER_MOTOR_ROT);
    DogLog.log("Subsystems/Intake/Rollers/TargetSpeed (rps)", targetRollersRPS);
    DogLog.log("Subsystems/Intake/Rollers/AtTargetSpeed", atTargetSpeed());
    DogLog.log("Subsystems/Intake/Arm/AtTargetAngle", atTargetAngle());
    DogLog.log("Subsystems/Intake/Arm/AbsoluteEncoderRaw (rots)", getCancoderPositionRaw());
    DogLog.log("Subsystems/Intake/Arm/FusedCurrentPosition (degs)", getArmPosition().getDegrees());
    DogLog.log(
        "Subsystems/Intake/Arm/AbsoluteCurrentPosition (degs)",
        getArmUnfusedPosition().getDegrees());
    DogLog.log("Subsystems/Intake/Arm/TargetPosition (degs)", targetAngleDeg);

    SmartDashboard.putNumber("Arm Angle", getArmPosition().getDegrees());
  }
}
