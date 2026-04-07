package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FuelGaugeDetection.FuelGauge;
import frc.robot.Constants.FuelGaugeDetection.GaugeCalculationType;
import frc.robot.util.LoggedTalonFX;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

// spotless is clapping out
public class HopperSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final BooleanSupplier redside;

  private final LoggedTalonFX hopperMotor1, hopperMotor2, hopper;
  private double targetSurfaceSpeedMps = 0.0;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  public HopperSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;

    CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Hopper.STATOR_LIMIT_AMPS)
            .withSupplyCurrentLimit(Constants.Hopper.SUPPLY_LIMIT_AMPS);

    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Hopper.kP).withKV(Constants.Hopper.kV);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    ClosedLoopRampsConfigs clrc = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.3);

    hopperMotor1 =
        new LoggedTalonFX(
            "HopperFloorMain", Constants.Hopper.MOTOR_1_PORT, Constants.Swerve.CAN_BUS);
    hopperMotor2 =
        new LoggedTalonFX(
            "HopperFloorFollower", Constants.Hopper.MOTOR_2_PORT, Constants.Swerve.CAN_BUS);
    hopper = hopperMotor1;

    TalonFXConfiguration hopperConfig =
        new TalonFXConfiguration()
            .withSlot0(s0c)
            .withCurrentLimits(currentLimitConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withClosedLoopRamps(clrc);

    TalonFXConfigurator hopperMotor1Config = hopperMotor1.getConfigurator();
    TalonFXConfigurator hopperMotor2Config = hopperMotor2.getConfigurator();
    hopperMotor1Config.apply(hopperConfig);
    hopperMotor2Config.apply(hopperConfig);

    hopperMotor2.setControl(new Follower(hopper.getDeviceID(), MotorAlignmentValue.Aligned));

    DogLog.log("Subsystems/Hopper/Gains/kP", Constants.Hopper.kP);
    DogLog.log("Subsystems/Hopper/Gains/kV", Constants.Hopper.kV);
  }

  // Ruth's version
  public void runHopperMps(DoubleSupplier targetSurfaceSpeedMps, BooleanSupplier readyToRun) {
    if (readyToRun.getAsBoolean()) {
      this.targetSurfaceSpeedMps = targetSurfaceSpeedMps.getAsDouble();
      hopper.setControl(
          m_velocityRequest.withVelocity(
              this.targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTS_PER_FLOOR_METER));
    } else {
      stop();
    }
  }

  public void runHopperMps(double targetSurfaceSpeedMps) {
    this.targetSurfaceSpeedMps = targetSurfaceSpeedMps;
    hopper.setControl(
        m_velocityRequest.withVelocity(
            targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTS_PER_FLOOR_METER));
  }

  public void stop() {
    targetSurfaceSpeedMps = 0.0;
    hopper.stopMotor();
  }

  public double getFloorSpeedMPS() {
    return hopper.getCachedVelocityRps() * Constants.Hopper.FLOOR_METERS_PER_MOTOR_ROT;
  }

  public double getAgitatorSpeedRPS() {
    return hopper.getCachedVelocityRps() * Constants.Hopper.AGITATOR_ROTS_PER_MOTOR_ROT;
  }

  public boolean atTargetSpeed() {
    return Math.abs(getFloorSpeedMPS() - targetSurfaceSpeedMps)
        <= Constants.Hopper.FLOOR_SPEED_TOLERANCE_MPS;
  }

  public boolean isHopperSufficientlyEmpty(FuelGaugeDetection fuelGaugeDetection) {
    return fuelGaugeDetection != null
        ? fuelGaugeDetection.GaugeLessThanEqualTo(
            GaugeCalculationType.SMOOTHED_MULTIPLE_BALLS, FuelGauge.LOW)
        : false;
  }

  // Does not stop the Hopper when interrupted
  public Command runHopperCommand() {
    return runOnce(() -> runHopperMps(Constants.Hopper.TARGET_SURFACE_SPEED_MPS));
  }

  // Does not stop the Hopper when interrupted
  public Command runHopperCommand(double targetSurfaceSpeedMps) {
    return runOnce(() -> runHopperMps(targetSurfaceSpeedMps));
  }

  // Stops the Hopper when interrupted
  public Command runHopperUntilInterruptedCommand() {
    return startEnd(() -> runHopperMps(Constants.Hopper.TARGET_SURFACE_SPEED_MPS), this::stop);
  }

  // Stops the Hopper when interrupted
  // Ruth's Version
  public Command runHopperUntilInterruptedCommand(
      DoubleSupplier targetSurfaceSpeedMps, BooleanSupplier readyToRun) {
    return runEnd(() -> runHopperMps(targetSurfaceSpeedMps, readyToRun), this::stop);
  }

  public Command runHopperUntilInterruptedCommand(double targetSurfaceSpeedMps) {
    return runEnd(() -> runHopperMps(targetSurfaceSpeedMps), this::stop);
  }

  // public double grabHopperRecommendedSpeed(double distanceToTarget) {
  //   return Constants.Hopper.HOPPER_SPEED_MAP.get(distanceToTarget);
  // }

  @Override
  public void periodic() {
    DogLog.log(
        "Subsystems/Hopper/CurrentSurfaceSpeed (mps)",
        hopper.getCachedVelocityRps() * Constants.Hopper.FLOOR_METERS_PER_MOTOR_ROT);
    DogLog.log("Subsystems/Hopper/TargetSurfaceSpeed (mps)", targetSurfaceSpeedMps);
    DogLog.log("Subsystems/Hopper/AtTargetSpeed", atTargetSpeed());
    DogLog.log(
        "Subsystems/Hopper/TargetMotorSpeed (rps)",
        targetSurfaceSpeedMps * Constants.Hopper.MOTOR_ROTS_PER_FLOOR_METER);
    DogLog.log("Subsystems/Hopper/CurrentMotorSpeed (rps)", hopper.getCachedVelocityRps());
  }
}
