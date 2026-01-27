package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class HopperSubsystem extends SubsystemBase {
  private static HopperSubsystem instance;

  private static double tolerance = 0.1;
  private final LoggedTalonFX motor;

  public static HopperSubsystem getInstance() {
    if (instance == null) {
      instance = new HopperSubsystem();
    }
    return instance;
  }

  public HopperSubsystem() {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Hopper.kP)
            .withKI(Constants.Hopper.kI)
            .withKD(Constants.Hopper.kD);

    motor = new LoggedTalonFX(Constants.Hopper.MOTOR_PORT);

    MotorOutputConfigs moc =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    motor.getConfigurator().apply(s0c);
    motor.getConfigurator().apply(clc);
    motor.getConfigurator().apply(moc);
  }

  public void runHopper() {
    motor.setControl(
        new VelocityVoltage(
            Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC
                / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL));
  }

  public void stop() {
    motor.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {
    return motor.getVelocity().getValueAsDouble()
            - Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC
                / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL
        <= tolerance;
  }

  @Override
  public void periodic() {
    DogLog.log("HopperSubsystem/Speed", motor.getVelocity().getValueAsDouble());
    DogLog.log("HopperSubsystem/AtSpeed", atSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
