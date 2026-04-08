package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
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
  private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
  private final CANcoder hoodEncoder;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);
  private double targetShooterSpeedRPS = 0;
  private double hoodTargetDeg = 0;

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
    this.drivetrain = drivetrain;
    this.redside = redside;
    CANBus canbus = Constants.Swerve.CAN_BUS;
    warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.Rollers.WARMUP_1_ID, canbus);
    warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.Rollers.WARMUP_2_ID, canbus);
    warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.Rollers.WARMUP_3_ID, canbus);
    shooter = warmup3;

    // TODO: fix hood id constant
    hood =
        new LoggedTalonFX("ShooterHood", Constants.Shooter.Hood.HOOD_ID, Constants.Swerve.CAN_BUS);

    Slot0Configs rollersS0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.Rollers.KP)
            .withKV(Constants.Shooter.Rollers.KV)
            .withKS(Constants.Shooter.Rollers.KS);

    CurrentLimitsConfigs rollersClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.Rollers.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.Rollers.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs rollersOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    // VoltageConfigs rollersVoltageOutputConfigs = new VoltageConfigs().withPeakReverseVoltage(1.0);

    TalonFXConfiguration rollersConfig = new TalonFXConfiguration();
    rollersConfig.Slot0 = rollersS0c;
    rollersConfig.CurrentLimits = rollersClc;
    rollersConfig.MotorOutput = rollersOutputConfigs;
    // rollersConfig.Voltage = rollersVoltageOutputConfigs;

    warmup1.getConfigurator().apply(rollersConfig);
    warmup2.getConfigurator().apply(rollersConfig);
    warmup3.getConfigurator().apply(rollersConfig);

    Follower follower =
        new Follower(Constants.Shooter.Rollers.WARMUP_3_ID, MotorAlignmentValue.Aligned);
    warmup1.setControl(follower);
    warmup2.setControl(follower);

    hoodEncoder = new CANcoder(Constants.Shooter.Hood.ENCODER_PORT, Constants.Swerve.CAN_BUS);

    Slot0Configs hoodS0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.Hood.KP)
            .withKI(Constants.Shooter.Hood.KI)
            .withKV(Constants.Shooter.Hood.KV)
            .withKS(Constants.Shooter.Hood.KS)
            .withKG(Constants.Shooter.Hood.KG)
            .withKD(Constants.Shooter.Hood.KD)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    CurrentLimitsConfigs hoodClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.Hood.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.Hood.SUPPLY_CURRENT_LIMIT);

    MotorOutputConfigs hoodMotorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

    FeedbackConfigs hoodFeedbackConfigs =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withSensorToMechanismRatio(Constants.Shooter.Hood.ENCODER_ROTS_PER_HOOD_ROT)
            .withRotorToSensorRatio(Constants.Shooter.Hood.MOTOR_ROTS_PER_ENCODER_ROT);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0 = hoodS0c;
    hoodConfig.CurrentLimits = hoodClc;
    hoodConfig.MotorOutput = hoodMotorOutputConfigs;
    hoodConfig.Feedback = hoodFeedbackConfigs;

    hood.getConfigurator().apply(hoodConfig);

    MagnetSensorConfigs hoodCANcoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Shooter.Hood.ENCODER_OFFSET));

    hoodEncoder.getConfigurator().apply(hoodCANcoderConfig);

    // Roller gains
    DogLog.log("Subsystems/Shooter/Gains/kP", Constants.Shooter.Rollers.KP);
    DogLog.log("Subsystems/Shooter/Gains/kV", Constants.Shooter.Rollers.KV);
    DogLog.log("Subsystems/Shooter/Gains/kS", Constants.Shooter.Rollers.KS); // NEW

    // Hood gains
    DogLog.log("Subsystems/Shooter/Hood/Gains/kP", Constants.Shooter.Hood.KP);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kS", Constants.Shooter.Hood.KS);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kG", Constants.Shooter.Hood.KG);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kV", Constants.Shooter.Hood.KV);
    DogLog.log("Subsystems/Shooter/Hood/Gains/kD", Constants.Shooter.Hood.KD);
  }

  public void setHoodPosition(double degrees) {
    hoodTargetDeg = degrees;
    hood.setControl(
        m_positionRequest.withPosition(
            MathUtil.clamp(
                    degrees,
                    Constants.Shooter.Hood.MIN_HOOD_POSITION,
                    Constants.Shooter.Hood.MAX_HOOD_POSITION)
                / 360.0));
  }

  public Rotation2d getHoodPosition() {
    return new Rotation2d(Units.rotationsToRadians(hood.getCachedPositionRotations()));
  }

  public void stopHood() {
    hoodTargetDeg = hood.getCachedPositionRotations() * 360.0;
    setHoodPosition(hoodTargetDeg);
  }

  // public boolean hoodAtTarget() {
  //   return Math.abs((hood.getCachedPositionRotations() * 360.0) - hoodTargetDeg)
  //       <= Constants.Shooter.Hood.HOOD_TOLERANCE_DEG;
  // }

  public double getHoodCancoderPositionRaw() {
    return hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // public Rotation2d getHoodUnfusedPosition() {
  //   return new Rotation2d(
  //       Units.rotationsToRadians(
  //           getHoodCancoderPositionRaw() * Constants.Shooter.Hood.HOOD_ROTS_PER_ENCODER_ROT));
  // }

  public void setShooterSpeedRPS(double shooterSpeedRPS) {
    targetShooterSpeedRPS = shooterSpeedRPS;
    shooter.setControl(
        m_velocityRequest.withVelocity(
            targetShooterSpeedRPS * Constants.Shooter.Rollers.MOTOR_ROTS_PER_WHEEL_ROT));
  }

  public void stopShooter() {
    targetShooterSpeedRPS = 0;
    shooter.stopMotor();
  }

  public boolean isAtSpeed() {
    if (shooter.getCachedVelocityRps() == 0) {
      return false;
    }
    return Math.abs(targetShooterSpeedRPS - getCurrentShooterWheelSpeedRPS())
        <= Constants.Shooter.Rollers.TOLERANCE_RPS;
  }

  public double getCurrentShooterWheelSpeedRPS() {
    return shooter.getCachedVelocityRps() * Constants.Shooter.Rollers.WHEEL_ROTS_PER_MOTOR_ROT;
  }

  public double getTargetShooterWheelSpeedRPS() {
    return targetShooterSpeedRPS;
  }

  public double grabTargetShootingSpeed(double distanceToTarget) {
    return Constants.Shooter.ROLLER_SPEED_MAP.get(distanceToTarget)-0.4;
  }

  public double grabPassingShootingSpeed(double distanceToTarget) {
    return Constants.Shooter.PASSING_ROLLER_SPEED_MAP.get(distanceToTarget);
  }

  public double grabTargetHoodAngle(double distanceToTarget) {
    return Constants.Shooter.HOOD_ANGLE_MAP.get(distanceToTarget);
  }

  public boolean isShooterReady() {
    return isAtSpeed(); // && hoodAtTarget()
  }

  // Commands
  public Command shootAtSpeedCommand() {
    return runEnd(() -> setShooterSpeedRPS(44.2), this::stopShooter);
  }

  public Command shootAtSpeedHoodCommand(double shooterSpeedRPS, double hoodAngle) {
    return runEnd(
        () -> {
          setShooterSpeedRPS(shooterSpeedRPS);
          setHoodPosition(hoodAngle);
        },
        this::stopShooter);
  }

  public Command shootAtSpeedCommand(double shooterSpeedRPS) {
    return runEnd(() -> setShooterSpeedRPS(shooterSpeedRPS), this::stopShooter);
  }

  public Command shootAtSpeedHoodCommand(DoubleSupplier shooterSpeedRPS, DoubleSupplier hoodAngle) {
    return runEnd(
        () -> {
          setShooterSpeedRPS(shooterSpeedRPS.getAsDouble());
          setHoodPosition(hoodAngle.getAsDouble());
          DogLog.log("SkibSpeed", shooterSpeedRPS.getAsDouble());
          DogLog.log("SkibAngle", hoodAngle.getAsDouble());
        },
        this::stopShooter);
  }

  public Command shootAtSpeedCommand(DoubleSupplier shooterSpeedRPS) {
    return runEnd(
        () -> {
          DogLog.log("Subsystems/Shooter/ShootingSpeedRN", shooterSpeedRPS.getAsDouble());
          this.setShooterSpeedRPS(shooterSpeedRPS.getAsDouble());
        },
        this::stopShooter);
  }

  public void moveHoodWithVoltage(double volts) {
    // hood.setControl(m_voltageRequest.withOutput(Constants.Shooter.Hood.ZERO_VOLTAGE));
    hood.setControl(m_voltageRequest.withOutput(volts));
  }

  public Command moveHoodWithVoltageCommand(double volts) {
    return runEnd(() -> this.moveHoodWithVoltage(volts), this::stopShooter);
  }

  @Override
  public void periodic() {
    // Shooter velocity information
    DogLog.log("Subsystems/Shooter/TargetWheelSpeed (rps)", getTargetShooterWheelSpeedRPS());
    DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)", getCurrentShooterWheelSpeedRPS());

    // Shooter status information
    DogLog.log("Subsystems/Shooter/AtTargetSpeed", isAtSpeed());
    // DogLog.log("Subsystems/Shooter/AtTargetAngle", hoodAtTarget());
    DogLog.log("Subsystems/Shooter/ShooterReady", isShooterReady());

    // Hood position information
    double currentHoodDeg = hood.getCachedPositionRotations() * 360.0;
    DogLog.log("Subsystems/Shooter/Hood/CurrentPositionDeg", currentHoodDeg);
    DogLog.log("Subsystems/Shooter/Hood/TargetPositionDeg", hoodTargetDeg);
    DogLog.log("Subsystems/Shooter/Hood/ErrorDeg", currentHoodDeg - hoodTargetDeg);
    DogLog.log(
        "Subsystems/Shooter/Hood/UnfusedCANcoderPositionDeg",
        getHoodCancoderPositionRaw() * 360.0); // CANCAoder reading before fusion

    // Targeting
    Pose2d target = redside.getAsBoolean() ? Landmarks.RED_HUB : Landmarks.BLUE_HUB;
    double distanceMeters = Targeting.distMeters(drivetrain, target);

    DogLog.log("Subsystems/Shooter/Targeting/DistanceMeters", distanceMeters);

    // TODO: Cache this value
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetPlusLead",
        Targeting.positionToTarget(target, drivetrain));
    DogLog.log(
        "Subsystems/Shooter/Targeting/ShootingSpeed", Targeting.shootingSpeed(target, drivetrain));
    DogLog.log(
        "Subsystems/Shooter/Targeting/DistanceMeters", Targeting.distMeters(drivetrain, target));
    DogLog.log(
        "Subsystems/Shooter/Targeting/TargetAngle", Targeting.targetAngle(target, drivetrain));
    DogLog.log(
        "Subsystems/Shooter/Targeting/IsPointing", Targeting.pointingAtTarget(target, drivetrain));

    DogLog.log(
        "Subsystems/Shooter/Targeting/TimeOfFlight",
        Targeting.newtonTargetingInfo(target, drivetrain).getToF());
    // DogLog.log("Subsystems/Shooter/CurrentSpeed (rps)",
    // shooter.getVelocity().getValueAsDouble());
    // "Subsystems/Shooter/Targeting/MappedShooterSpeedRPS",
    // grabTargetShootingSpeed(distanceMeters));
    DogLog.log(
        "Subsystems/Shooter/Targeting/MappedHoodAngleDeg", grabTargetHoodAngle(distanceMeters));
  }
}
