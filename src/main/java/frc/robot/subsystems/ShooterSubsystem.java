// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Landmarks;
import frc.robot.util.LoggedTalonFX;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final BooleanSupplier redside;

  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private double targetBallSpeed = 0; // this needs to be consistent
  private static final double TOLERANCE_RPS = 2.0; // tolerance in rotations per second

  private final double coefficient = 1;

  // Simulation objects
  // private TalonFXSimState shooterSimState;
  private DCMotorSim shooterMechanismSim;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;

    warmUpMotor1 =
        new LoggedTalonFX(
            "ShooterWarmUp1",
            Constants.Shooter.WARMUP_1_ID,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    warmUpMotor2 =
        new LoggedTalonFX(
            "ShooterWarmUp2",
            Constants.Shooter.WARMUP_2_ID,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    warmUpMotor3 =
        new LoggedTalonFX(
            "ShooterWarmUp3",
            Constants.Shooter.WARMUP_3_ID,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    shooter = warmUpMotor3;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD)
            .withKV(Constants.Shooter.KV)
            .withKA(Constants.Shooter.KA);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    VoltageConfigs vConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

    // Apply full TalonFXConfiguration to ensure factory defaults
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = s0c;
    config.CurrentLimits = clc;
    config.MotorOutput = motorOutputConfigs;
    config.Voltage = vConfigs;

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(config);
    m2config.apply(config);
    m3config.apply(config);

    // Set motors 1 and 2 to follow motor 3 (the leader)
    Follower follower = new Follower(Constants.Shooter.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmUpMotor1.setControl(follower);
    warmUpMotor2.setControl(follower);

    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.KP);
    DogLog.log("Subsystems/Shooter/Gains/kI", Constants.Shooter.KI);
    DogLog.log("Subsystems/Shooter/Gains/kD", Constants.Shooter.KD);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.KV);
    DogLog.log("Subsystems/Shooter/Gains/kA", Constants.Shooter.KA);

    // if (RobotBase.isSimulation()) setupSimulation();
  }

  // private void setupSimulation() {
  // shooterSimState = warmUpMotor3.getSimState();
  // shooterSimState.Orientation = ChassisReference.CounterClockwise_Positive;
  // shooterSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

  // // Use a SINGLE motor model since only Motor 3 is actively controlled
  // var singleKrakenGearbox = DCMotor.getKrakenX60Foc(1);

  // shooterMechanismSim =
  // new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(
  // singleKrakenGearbox,
  // Constants.Shooter.SHOOTER_SIM_MOI_KG_M2, // MOI of entire coupled system
  // Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS), // Motor 3 → Shooter wheel
  // (1.25)
  // singleKrakenGearbox);
  // }

  // from linear speed in ft/sec to motor rps
  public double calculateFtPSToRPS(double speedFtPS) {
    return speedFtPS
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI)
        * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
  }

  // from motor rps to linear speed in ft/sec
  public double calculateRPSToFtPS(double rps) {
    return (rps / Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS)
        * Constants.Shooter.SHOOTER_WHEEL_DIAMETER
        * Math.PI;
    // return rps / 12
    // * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER * Math.PI)
    // / Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
  }

  // speed based on shooter wheel which is the one flinging the ball with a max of
  // 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2
  // because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setBallSpeed(double ballSpeed) {
    targetBallSpeed = ballSpeed;
    shooter.setControl(
        m_velocityRequest.withVelocity(calculateFtPSToRPS(targetBallSpeed * coefficient)));
  }

  public void stopShooter() {
    setBallSpeed(0);
  }

  public boolean isAtSpeed() {
    if (shooter.getCachedVelocityRps() == 0) {
      return false;
    }
    return Math.abs(calculateFtPSToRPS(targetBallSpeed) - shooter.getCachedVelocityRps())
        <= TOLERANCE_RPS;
  }

  public double getCurrentBallSpeedFtPS() {
    return calculateRPSToFtPS(shooter.getCachedVelocityRps());
  }

  public double getTargetBallSpeedFtPS() {
    return targetBallSpeed;
  }

  public double grabTargetShootingSpeed(double distanceToTarget) {
    return Constants.Shooter.MOTOR_SPEED_FPS_FOR_DISTANCE_METERS_CENTER_TO_CENTER_INTERMAP.get(
        distanceToTarget);
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setBallSpeed(Constants.Shooter.SHOOT_FOR_AUTO), this::stopShooter);
  }

  public Command shootAtSpeedCommand(double ballSpeed) {
    return runEnd(() -> setBallSpeed(ballSpeed), this::stopShooter);
  }

  public Command shootAtSpeedCommand(DoubleSupplier ballSpeed) {
    DogLog.log("ShootingSpeedRN", ballSpeed.getAsDouble());
    return runEnd(() -> this.setBallSpeed(ballSpeed.getAsDouble()), this::stopShooter);
  }

  public Command shootAtSpeedCommand(DoubleSubscriber ballSpeed) {
    DogLog.log("ShootingSpeedRN", ballSpeed.get());
    return runEnd(() -> this.setBallSpeed(ballSpeed.get()), this::stopShooter);
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Shooter/TargetSpeed (fps)", getTargetBallSpeedFtPS());
    DogLog.log("Subsystems/Shooter/AtTargetSpeed", isAtSpeed());
    DogLog.log("Subsystems/Shooter/CurrentSpeed (fps)", getCurrentBallSpeedFtPS());

    Pose3d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetPlusLead",
        new Pose2d(
            Targeting.positionToTarget(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
                .x,
            Targeting.positionToTarget(
                    target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION)
                .y,
            new Rotation2d()));
    DogLog.log(
        "Subsystems/Shooter/Targeting/ShootingSpeed",
        Targeting.shootingSpeed(
            target, drivetrain, Constants.Shooter.TARGETING_CALCULATION_PRECISION));
    DogLog.log(
        "Subsystems/Shooter/Targeting/DistanceMeters", Targeting.distMeters(drivetrain, target));
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetAngle", Targeting.targetAngle(target, drivetrain));
    DogLog.log(
        "Subsystems/Shooter/Targeting/IsPointing", Targeting.pointingAtTarget(target, drivetrain));
    DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)", shooter.getVelocity().getValueAsDouble());
  }

  // @Override
  // public void simulationPeriodic() {
  // if (shooterSimState == null || shooterMechanismSim == null) {
  // return;
  // }

  // // 1) Supply voltage to all three motor sims
  // double batteryV = RobotController.getBatteryVoltage();
  // shooterSimState.setSupplyVoltage(batteryV);

  // // 2) Read applied motor voltage from leader (motor3) and step mechanism
  // plant
  // // Since motor1 and motor2 follow motor3, we only read motor3's voltage
  // double appliedMotorVoltageVolts =
  // shooterSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

  // shooterMechanismSim.setInputVoltage(appliedMotorVoltageVolts);
  // shooterMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

  // // 3) Mechanism-side sim -> rotor-side sensor state
  // // DCMotorSim tracks the shooter wheel mechanism (after gear reduction)
  // double shooterWheelVelocityRotationsPerSecond =
  // shooterMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
  // double shooterWheelPositionRotations =
  // shooterMechanismSim.getAngularPositionRotations();

  // // Convert mechanism rotations to motor rotor rotations
  // double motorRotorPositionRotations =
  // shooterWheelPositionRotations * Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;
  // double motorRotorVelocityRotationsPerSecond =
  // shooterWheelVelocityRotationsPerSecond *
  // Constants.Shooter.MOTOR_ROTS_PER_WHEEL_ROTS;

  // shooterSimState.setRawRotorPosition(motorRotorPositionRotations);
  // shooterSimState.setRotorVelocity(motorRotorVelocityRotationsPerSecond);

  // // 4) Battery sag model
  // // Sum the supply current from all three motors
  // double loadedBatteryVoltageVolts =
  // BatterySim.calculateDefaultBatteryLoadedVoltage(shooterSimState.getSupplyCurrent()
  // * 3);
  // RoboRioSim.setVInVoltage(loadedBatteryVoltageVolts);
  // }
}
