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

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;

  private static double tolerance = 0.1;
  private final LoggedTalonFX muscleUpMotor, sitUpMotor, pullUpMotorL, pullUpMotorR;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  public ClimberSubsystem() {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);

    Slot0Configs s0c = new Slot0Configs();

    motor = new LoggedTalonFX(-1);

    
  }

  @Override
  public void periodic() {
    DogLog.log("ShooterSubsystem/Speed", motor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}