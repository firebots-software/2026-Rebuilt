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

  private static double tolerance = Constants.Hopper.TOLERANCE_MOTOR_ROTS_PER_SEC;
  private final LoggedTalonFX motor;
  private double targetSpeed = 0;

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

  public void runHopper(double speed) {
    targetSpeed = speed;
    motor.setControl(
        new VelocityVoltage(speed / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL));
  }

  public void stop() {
    targetSpeed = 0;
    motor.setControl(new VelocityVoltage(0));
  }

  public boolean atSpeed() {
    return motor.getVelocity().getValueAsDouble()
            - targetSpeed / Constants.Hopper.MOTOR_ROTS_TO_METERS_OF_PULLEY_TRAVERSAL
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
