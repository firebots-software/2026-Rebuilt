package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private double targetSpeed = 40d;
  private double tolerance = 1;
  private static IntakeSubsystem instance;
  private LoggedTalonFX motor;

  public IntakeSubsystem() {
    // change ports as needed
    motor = new LoggedTalonFX(33);
    Slot0Configs s0c = new Slot0Configs().withKP(0.1).withKI(0).withKD(0);

    MotorOutputConfigs moc = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive);

    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(50.0);

    TalonFXConfigurator mConfig = motor.getConfigurator();

    mConfig.apply(s0c);
    mConfig.apply(moc);
    mConfig.apply(clc);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null)
      instance = new IntakeSubsystem();
    return instance;
  }

  public void run(double speed) {
    motor.setControl(new VelocityVoltage(speed * 2d).withFeedForward(0.1));
  }

  public void stop() {
    motor.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {
    return Math.abs(motor.getVelocity().getValueAsDouble() - (targetSpeed * 2d)) <= tolerance;
  }

  @Override
  public void periodic() {
    DogLog.log("DogLog/intake/motorVelocity", motor.getVelocity().getValueAsDouble());
    DogLog.log("DogLog/intake/targetSpeed", targetSpeed * 2d);
    DogLog.log("DogLog/intake/atSpeed", atSpeed());
  }
}
