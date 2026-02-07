package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class TestArm extends SubsystemBase {
  private final LoggedTalonFX motor;
  private final Servo brake;

  public double targetAngle = 50;

  public TestArm() {
    motor = new LoggedTalonFX(1);
    brake = new Servo(0);

    CurrentLimitsConfigs regClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.DEFAULT_STATOR_CURRENT)
            .withSupplyCurrentLimit(Constants.Climber.DEFAULT_SUPPLY_CURRENT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Climber.KP)
            .withKI(Constants.Climber.KI)
            .withKD(Constants.Climber.KD);

    MotorOutputConfigs moc = new MotorOutputConfigs();

    TalonFXConfigurator motorConfig = motor.getConfigurator();

    motorConfig.apply(moc);
    motorConfig.apply(regClc);
  }

  public void setPos() {
    motor.setPosition(50);
  }

  public boolean isAtPos() {
    return Math.abs(motor.getPosition().getValueAsDouble() - targetAngle) <= 1;
  }

  public void brake() {
    motor.stopMotor();
    brake.setAngle(50);
  }

  
  // Commands

  public Command SetPos() {
    return Commands.runOnce(() -> this.setPos(), this);
  }

  public Command BrakeCommand() {
    return Commands.runOnce(() -> this.brake(), this);
  }

  public Command SetAngle() {
    return Commands.sequence(SetPos(), BrakeCommand());
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
