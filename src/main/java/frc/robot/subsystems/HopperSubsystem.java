package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class HopperSubsystem extends SubsystemBase {

  private final LoggedTalonFX motor;
  private double targetSpeed = 0;

  // Simulation objects
  private TalonFXSimState motorSimState;
  private DCMotorSim physicsSim;

  public HopperSubsystem() {
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.HOPPER_STATOR_LIMIT)
            .withSupplyCurrentLimit(Constants.Hopper.HOPPER_SUPPLY_LIMIT);

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

    // Initialize simulation
    if (RobotBase.isSimulation()) {
      setupSimulation();
    }
  }

  private void setupSimulation() {
    // Get SimState from the underlying TalonFX
    // If LoggedTalonFX extends TalonFX: motor.getSimState()
    // If LoggedTalonFX wraps TalonFX: motor.getTalonFX().getSimState() or similar
    motorSimState = motor.getSimState();

    // Set mechanical orientation (usually CCW+ for single motor mechanisms)
    motorSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    // Set motor type for accurate physics
    motorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Create physics model for the hopper mechanism
    // Assuming hopper is a flywheel-like mechanism or pulley system
    // Adjust these constants based on your actual mechanism
    double momentOfInertiaKgM2 = 0.001; // Adjust based on your pulley/mass
    double gearRatio = 1.0; // Adjust if you have gearing

    var gearbox = DCMotor.getKrakenX60Foc(1);
    
    physicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(gearbox, momentOfInertiaKgM2, gearRatio),
        gearbox
    );
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
        <= Constants.Hopper.TOLERANCE_MOTOR_ROTS_PER_SEC;
  }

  // Commands
  public Command RunHopper(double speed) {
    return Commands.runEnd(
        () -> this.runHopper(Constants.Hopper.TARGET_PULLEY_SPEED_M_PER_SEC), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("HopperSubsystem/Speed", motor.getVelocity().getValueAsDouble());
    DogLog.log("HopperSubsystem/AtSpeed", atSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // Update supply voltage (battery simulation)
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the voltage the motor controller is applying
    var motorVoltage = motorSimState.getMotorVoltageMeasure();

    // Update physics simulation with the applied voltage
    physicsSim.setInputVoltage(motorVoltage.in(edu.wpi.first.units.Units.Volts));
    physicsSim.update(0.020); // 20ms update period

    // Feed physics results back to CTRE simulation
    // Note: DCMotorSim returns mechanism position/velocity (after gear ratio)
    // TalonFX expects rotor position/velocity (before gear ratio)
    // If gear ratio is 1.0, they're the same
    double gearRatio = 1.0; // Adjust if different
    
    motorSimState.setRawRotorPosition(
        physicsSim.getAngularPosition().times(gearRatio)
    );
    motorSimState.setRotorVelocity(
        physicsSim.getAngularVelocity().times(gearRatio)
    );
  }
}