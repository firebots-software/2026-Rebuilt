package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private double tolerance = 1;

  private static IntakeSubsystem instance;
  private LoggedTalonFX armMotor, intakeMotor;
  private DutyCycleEncoder encoder;

  public IntakeSubsystem() {
    intakeMotor = new LoggedTalonFX(Constants.Intake.intakeMotor.port);
    armMotor = new LoggedTalonFX(Constants.Intake.Arm.armMotor.port);

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

    encoder = new DutyCycleEncoder(Constants.Intake.Arm.encoderPort);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) instance = new IntakeSubsystem();
    return instance;
  }

  public void run(double speed) {
    intakeMotor.setControl(new VelocityVoltage(speed).withFeedForward(0.1));
  }

  public void stop() {
    intakeMotor.setControl(new VelocityVoltage(0));
  }

  public void setArmDegrees(double angle) {
    // PositionTorqueCurrentFOC might not be the right control request
    armMotor.setControl(new PositionTorqueCurrentFOC(angle).withFeedForward(0.1));
  }

  public double getAbsolutePosition() {
    return encoder.get();
  }

  public boolean atSpeed() {
    return Math.abs(
            intakeMotor.getVelocity().getValueAsDouble() - Constants.Intake.intakeTargetSpeed)
        <= tolerance;
  }

  @Override
  public void periodic() {
    DogLog.log("Intake/motorVelocity", intakeMotor.getVelocity().getValueAsDouble());
    DogLog.log("Intake/targetSpeed", Constants.Intake.intakeTargetSpeed);
    DogLog.log("Intake/atSpeed", atSpeed());
    DogLog.log("Intake/motorCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/absEncoderPos", encoder.get());
  }
}
