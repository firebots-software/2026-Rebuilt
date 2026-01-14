package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private double targetSpeed = 20d * 2;
  private static IntakeSubsystem instance;
  private LoggedTalonFX motor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public IntakeSubsystem() {
    // change ports as needed
    motor = new LoggedTalonFX(33);

    Slot0Configs s0c = new Slot0Configs().withKP(.1).withKI(0).withKD(0);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    motor.updateCurrentLimits(30, 30);

    TalonFXConfigurator mConfig = motor.getConfigurator();

    mConfig.apply(s0c);
    mConfig.apply(moc);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) instance = new IntakeSubsystem();
    return instance;
  }

  public void setSpeed(double speed) {
    motor.setControl(velocityRequest.withVelocity(speed * -2d));
  }

  public void stop() {
    motor.setControl(velocityRequest.withVelocity(0));
  }

  @Override
  public void periodic() {
    DogLog.log("DogLog/intake/motorVelocity", motor.getVelocity().getValueAsDouble());
    DogLog.log("DogLog/intake/targetSpeed", targetSpeed);
  }
}
