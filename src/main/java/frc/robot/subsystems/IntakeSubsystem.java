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
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX armMotor, intakeMotor;
  private CANcoder cancoder;
  private double targetAngle;

  public IntakeSubsystem() {
    intakeMotor = new LoggedTalonFX(Constants.Intake.INTAKE_MOTOR.port);
    armMotor = new LoggedTalonFX(Constants.Intake.Arm.ARM_MOTOR.port);
    targetAngle = Constants.Intake.Arm.ARM_POS_INITIAL;

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
  }

  public void run(double speed) {
    intakeMotor.setControl(
        new VelocityVoltage(speed * Constants.Intake.MOTOR_ROTS_TO_INTAKE_ROTS)
            .withFeedForward(Constants.Intake.INTAKE_FEEDFORWARD));
  }

  public void stop() {
    intakeMotor.setControl(new VelocityVoltage(0));
  }

  public void setArmDegrees(double angle) {
    targetAngle = angle * Constants.Intake.Arm.MOTOR_ROTS_TO_ARM_DEGREES;
    // PositionTorqueCurrentFOC might not be the right control request
    armMotor.setControl(
        new PositionTorqueCurrentFOC(
                MathUtil.clamp(targetAngle, 0, Constants.Intake.Arm.ARM_DEGREES_UPPER_LIMIT))
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
    // uses the fusedcancoder (more accurate)
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
    DogLog.log("Subsystems/Intake-Rollers/Target Speed", Constants.Intake.INTAKE_TARGET_SPEED);
    DogLog.log("Subsystems/Intake-Rollers/At target speed", atSpeed());
    DogLog.log(
        "Subsystems/Intake-Rollers/Motor Velocity (rots/s)",
        intakeMotor.getVelocity().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake-Rollers/Motor Velocity (ft/s)",
        intakeMotor.getVelocity().getValueAsDouble()
            * Constants.Intake.INTAKE_ROTS_PER_SEC_TO_FEET_PER_SEC);
    DogLog.log(
        "Subsystems/Intake-Rollers/Motor Position (rots)",
        intakeMotor.getPosition().getValueAsDouble());
    DogLog.log(
        "Subsystems/Intake-Rollers/Motor Current (stator)",
        intakeMotor.getStatorCurrent().getValueAsDouble());

    // arm
    DogLog.log("Subsystems/Intake-Arm/CANcoder Position (degrees)", getCancoderPosition());
    DogLog.log("Subsystems/Intake-Arm/CANcoder Position (raw)", getCancoderPositionRaw());
    DogLog.log("Subsystems/Intake-Arm/Position (degrees)", getEncoderPosition());
    DogLog.log("Subsystems/Intake-Arm/Position (raw)", getEncoderPositionRaw());
    DogLog.log(
        "Subsystems/Intake-Arm/Motor Velocity (rots/s)", armMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Intake-Arm/Target Angle", targetAngle);
  }
}
