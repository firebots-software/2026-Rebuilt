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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;
  private LoggedTalonFX armMotor, intakeMotor;
  private CANcoder cancoder;
  private double targetAngle;

  public IntakeSubsystem() {
    intakeMotor = new LoggedTalonFX(Constants.Intake.intakeMotor.port);
    armMotor = new LoggedTalonFX(Constants.Intake.Arm.armMotor.port);
    targetAngle = Constants.Intake.Arm.armPosInitial;

    Slot0Configs intakeSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.intakeKV)
            .withKP(Constants.Intake.intakeKP)
            .withKI(Constants.Intake.intakeKI)
            .withKD(Constants.Intake.intakeKD);

    Slot0Configs armSlot0Configs =
        new Slot0Configs()
            .withKV(Constants.Intake.Arm.armKV)
            .withKP(Constants.Intake.Arm.armKP)
            .withKI(Constants.Intake.Arm.armKI)
            .withKD(Constants.Intake.Arm.armKD);

    CurrentLimitsConfigs intakeCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Intake.intakeStatorCurrentLimit)
            .withSupplyCurrentLimit(Constants.Intake.intakeSupplyCurrentLimit);

    CurrentLimitsConfigs armCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Intake.Arm.armStatorCurrentLimit);

    TalonFXConfigurator armMotorConfig = armMotor.getConfigurator();
    TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();

    armMotorConfig.apply(armSlot0Configs);
    armMotorConfig.apply(armCurrentLimitsConfigs);
    armMotorConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    intakeMotorConfig.apply(intakeSlot0Configs);
    intakeMotorConfig.apply(intakeCurrentLimitsConfigs);
    intakeMotorConfig.apply(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    // create a new FusedCANcoder class which combines data from the CANcoder and
    // the arm motor's encoder
    cancoder = new CANcoder(Constants.Intake.Arm.encoderPort);
    CANcoderConfiguration ccConfig = new CANcoderConfiguration();
    // zero the magnet
    ccConfig
        .MagnetSensor
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(Rotations.of(0.4));
    cancoder.getConfigurator().apply(ccConfig);

    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    fxConfig
        .Feedback
        .withFeedbackRemoteSensorID(cancoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(Constants.Intake.Arm.encoderRotsToArmRots)
        .withRotorToSensorRatio(Constants.Intake.Arm.armDegreesToMotorRots);
    armMotorConfig.apply(fxConfig);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) instance = new IntakeSubsystem();
    return instance;
  }

  public void run(double speed) {
    intakeMotor.setControl(
        new VelocityVoltage(speed * Constants.Intake.motorRotsToIntakeRots).withFeedForward(0.1));
  }

  public void stop() {
    intakeMotor.setControl(new VelocityVoltage(0));
  }

  public void setArmDegrees(double angle) {
    targetAngle = angle;
    // PositionTorqueCurrentFOC might not be the right control request
    armMotor.setControl(
        new PositionTorqueCurrentFOC(angle * Constants.Intake.Arm.motorRotsToArmDegrees)
            .withFeedForward(0.1));
  }

  public double getCancoderPosition() {
    // uses the cancoder's position data directly
    return cancoder.getAbsolutePosition().getValueAsDouble()
        / Constants.Intake.encoderRotsToIntakeRots;
  }

  public double getCancoderPositionRaw() {
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getEncoderPosition() {
    // uses the fusedcancoder (more accurate)
    return armMotor.getPosition().getValueAsDouble() / Constants.Intake.encoderRotsToIntakeRots;
  }

  public double getEncoderPositionRaw() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public boolean atSpeed() {
    return Math.abs(
            intakeMotor.getVelocity().getValueAsDouble() - Constants.Intake.intakeTargetSpeed)
        <= Constants.Intake.Arm.armToleranceDegrees;
  }

  @Override
  public void periodic() {
    DogLog.log("Intake/targetSpeed", Constants.Intake.intakeTargetSpeed);
    DogLog.log("Intake/atSpeed", atSpeed());

    DogLog.log("Intake/motorVelocity", intakeMotor.getVelocity().getValueAsDouble());
    DogLog.log(
        "Intake/motorVelocityFeet",
        intakeMotor.getVelocity().getValueAsDouble()
            * Constants.Intake.intakeRotsPerSecToFeetPerSec);
    DogLog.log("Intake/motorPos", intakeMotor.getPosition().getValueAsDouble());

    DogLog.log("Intake/motorCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/CANcoderPos", getCancoderPosition());
    DogLog.log("Intake/CANcoderPosRaw", getCancoderPositionRaw());
    DogLog.log("Intake/FusedCANcoderPos", getEncoderPosition());
    DogLog.log("Intake/FusedCANcoderPosRaw", getEncoderPositionRaw());

    DogLog.log("Intake/armMotorVelocity", armMotor.getVelocity().getValueAsDouble());
    DogLog.log("Intake/targetAngle", targetAngle);
  }
}
