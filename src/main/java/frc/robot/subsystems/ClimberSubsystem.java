package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ClimberSubsystem extends SubsystemBase {
  private final LoggedTalonFX muscleUpMotor, sitUpMotor, pullUpMotorR, pullUpMotorL;
  private double sitUpTargetDeg, muscleUpTargetDeg, pullUpTargetPosition;
  private final CANcoder sitUpEncoder;
  private final Servo brake;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);
  private final VoltageOut m_voltageOut = new VoltageOut(3.0);
  private final VoltageOut m_inwardsvoltageOut = new VoltageOut(3.0); // positive = inwards
  private final VoltageOut m_outwardsvoltageOut = new VoltageOut(-3.0);

  public ClimberSubsystem() {
    CurrentLimitsConfigs regClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.DEFAULT_STATOR_CURRENT)
            .withSupplyCurrentLimit(Constants.Climber.DEFAULT_SUPPLY_CURRENT);

    CurrentLimitsConfigs specialClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.SitUp.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Climber.SitUp.SUPPLY_CURRENT_LIMIT);

    Slot0Configs pullUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.PullUp.KP)
            .withKI(Constants.Climber.PullUp.KI)
            .withKD(Constants.Climber.PullUp.KD)
            .withKV(Constants.Climber.PullUp.KV)
            .withKG(Constants.Climber.PullUp.KG)
            .withGravityType(GravityTypeValue.Elevator_Static);

    Slot0Configs muscleUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.MuscleUp.KP)
            .withKI(Constants.Climber.MuscleUp.KI)
            .withKD(Constants.Climber.MuscleUp.KD);

    Slot0Configs sitUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.SitUp.KP)
            .withKI(Constants.Climber.SitUp.KI)
            .withKD(Constants.Climber.SitUp.KD);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotorOutputConfigs mocReversed =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Climber.SitUp.mmcV)
            .withMotionMagicAcceleration(Constants.Climber.SitUp.mmcA);

    muscleUpMotor =
        new LoggedTalonFX(
            "MuscleUp",
            Constants.Climber.MuscleUp.MOTOR_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    sitUpMotor =
        new LoggedTalonFX(
            "SitUp",
            Constants.Climber.SitUp.MOTOR_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    pullUpMotorR =
        new LoggedTalonFX(
            "PullUpRight",
            Constants.Climber.PullUp.MOTOR_R_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    pullUpMotorL =
        new LoggedTalonFX(
            "PullUpLeft",
            Constants.Climber.PullUp.MOTOR_L_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    pullUpMotorL.setControl(new Follower(pullUpMotorR.getDeviceID(), MotorAlignmentValue.Opposed));

    brake = new Servo(Constants.Climber.BRAKE_PORT);

    // create fusedcancoder
    sitUpEncoder =
        new CANcoder(
            Constants.Climber.SitUp.ENCODER_PORT, Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    MagnetSensorConfigs canCoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(Rotations.of(Constants.Climber.SitUp.ENCODER_OFFSET));
    sitUpEncoder.getConfigurator().apply(canCoderConfig);

    // setting configs to configurator
    TalonFXConfiguration muscleUpConfig = new TalonFXConfiguration();
    muscleUpConfig.Slot0 = muscleUps0c;
    muscleUpConfig.CurrentLimits = regClc;
    muscleUpConfig.MotorOutput = mocReversed;

    TalonFXConfiguration sitUpConfig = new TalonFXConfiguration();
    sitUpConfig.Slot0 = sitUps0c;
    sitUpConfig.CurrentLimits = specialClc;
    sitUpConfig.MotorOutput = moc;
    sitUpConfig.MotionMagic = mmc;
    sitUpConfig.Feedback.withFeedbackRemoteSensorID(sitUpEncoder.getDeviceID());
    sitUpConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
    sitUpConfig.Feedback.withSensorToMechanismRatio(
        Constants.Climber.SitUp.ENCODER_ROTS_PER_ARM_ROTS);
    sitUpConfig.Feedback.withRotorToSensorRatio(Constants.Climber.SitUp.MOTOR_ROTS_TO_ENCODER_ROTS);

    TalonFXConfiguration pullUpLeftConfig = new TalonFXConfiguration();
    pullUpLeftConfig.Slot0 = pullUps0c;
    pullUpLeftConfig.CurrentLimits = regClc;
    pullUpLeftConfig.MotorOutput = moc;

    TalonFXConfiguration pullUpRightConfig = new TalonFXConfiguration();
    pullUpRightConfig.Slot0 = pullUps0c;
    pullUpRightConfig.CurrentLimits = regClc;
    pullUpRightConfig.MotorOutput = moc;

    TalonFXConfigurator muscleUpMotorConfig = muscleUpMotor.getConfigurator();
    TalonFXConfigurator sitUpMotorConfig = sitUpMotor.getConfigurator();
    TalonFXConfigurator pullUpLeftMotorConfig = pullUpMotorL.getConfigurator();
    TalonFXConfigurator pullUpRightMotorConfig = pullUpMotorR.getConfigurator();

    muscleUpMotorConfig.apply(muscleUpConfig);
    sitUpMotorConfig.apply(sitUpConfig);
    pullUpLeftMotorConfig.apply(pullUpLeftConfig);
    pullUpRightMotorConfig.apply(pullUpRightConfig);

    DogLog.log("Subsystems/Climber/Gains/kP", Constants.Climber.KP);
    DogLog.log("Subsystems/Climber/Gains/kI", Constants.Climber.KI);
    DogLog.log("Subsystems/Climber/Gains/kD", Constants.Climber.KD);
  }

  public void setSitUpPosition(double degrees) {
    // sitUpTargetDeg = degrees * Constants.Climber.SitUp.DEGREES_OF_ARM_ROT_TO_MOTOR_ROTS;
    sitUpMotor.setControl(m_motionMagicRequest.withPosition(degrees));
  }

  public void setMuscleUpPosition(double degrees) {
    muscleUpTargetDeg = degrees * Constants.Climber.MuscleUp.MOTOR_ROTS_PER_ARM_DEGREES;
    muscleUpMotor.setControl(m_motionMagicRequest.withPosition(muscleUpTargetDeg));
  }

  public void setPullUpPosition(double metersFromZero) {
    pullUpTargetPosition = metersFromZero * Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS;
    pullUpMotorR.setControl(m_motionMagicRequest.withPosition(pullUpTargetPosition));
  }

  public boolean isSitUpAtPosition() {
    return Math.abs(
            sitUpMotor.getPosition().getValueAsDouble()
                    / Constants.Climber.SitUp.DEGREES_OF_ARM_ROT_TO_MOTOR_ROTS
                - sitUpTargetDeg)
        <= Constants.Climber.SitUp.SIT_UP_TOLERANCE;
  }

  public boolean isMuscleUpAtPosition() {
    return Math.abs(
            muscleUpMotor.getPosition().getValueAsDouble()
                    * Constants.Climber.MuscleUp.ARM_DEGREES_PER_MOTOR_ROTS
                - muscleUpTargetDeg)
        <= Constants.Climber.MuscleUp.MUSCLE_UP_TOLERANCE;
  }

  public boolean isPullUpAtPosition() {
    return Math.abs(
            pullUpMotorR.getPosition().getValueAsDouble()
                    / Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS
                - pullUpTargetPosition)
        <= Constants.Climber.PullUp.PULL_UP_TOLERANCE_METERS;
  }

  public double getSitUpPosInRotationsFromEncoder() {
    return sitUpEncoder.getAbsolutePosition().getValueAsDouble()
        / Constants.Climber.SitUp.ENCODER_ROTS_PER_ARM_ROTS;
  }

  public void stopSitUp() {
    sitUpTargetDeg = sitUpMotor.getPosition().getValueAsDouble();
    setSitUpPosition(sitUpTargetDeg);
  }

  public void stopPullUp() {
    pullUpTargetPosition = pullUpMotorR.getPosition().getValueAsDouble();
    setPullUpPosition(pullUpTargetPosition);
  }

  public Command stopPullUpCommand() {
    return runOnce(this::stopPullUp);
  }

  public void stopMuscleUp() {
    muscleUpTargetDeg = muscleUpMotor.getPosition().getValueAsDouble();
    setMuscleUpPosition(muscleUpTargetDeg);
  }

  public void unbrakeClimb() {
    brake.setAngle(Constants.Climber.UNBRAKE_ANGLE);
  }

  public void brakeClimb() {
    brake.setAngle(Constants.Climber.BRAKE_ANGLE);
    stopSitUp();
    stopPullUp();
    stopMuscleUp();
  }

  public void stopClimbWithoutBrake() {
    stopSitUp();
    stopPullUp();
    stopMuscleUp();
  }

  // Zeroing climb functions (only pull up because it doesn't have an encoder):

  public void reducePullUpCurrentLimits() {
    pullUpMotorR.updateCurrentLimits(6, 10);
    pullUpMotorL.updateCurrentLimits(6, 10);
  }

  public void reduceMuscleUpCurrentLimits() {
    muscleUpMotor.updateCurrentLimits(6, 10);
  }

  public void movePullUpDown() {
    pullUpMotorR.setControl(
        m_velocityRequest.withVelocity(Constants.Climber.PullUp.PULL_DOWN_VELOCITY));
  }

  public void movePullUpUp() {
    // pullUpMotorR.setControl(
    // m_velocityRequest.withVelocity(Constants.Climber.PullUp.PULL_UP_VELOCITY));
    pullUpMotorR.setControl(m_voltageOut);
  }

  public void moveMuscleUpDown() {
    // muscleUpMotor.setControl(
    // m_velocityRequest.withVelocity(Constants.Climber.MuscleUp.MUSCLEUP_DOWN_VELOCITY));
    muscleUpMotor.setControl(m_inwardsvoltageOut);
  }

  public void moveMuscleUpUp() {
    // muscleUpMotor.setControl(
    // m_velocityRequest.withVelocity(Constants.Climber.MuscleUp.MUSCLEUP_DOWN_VELOCITY));
    muscleUpMotor.setControl(m_outwardsvoltageOut);
  }

  public boolean checkPullUpCurrent() {
    double supplyCurrent = Math.abs(pullUpMotorR.getSupplyCurrent().getValue().magnitude());
    double statorCurrent = Math.abs(pullUpMotorR.getStatorCurrent().getValue().magnitude());

    return supplyCurrent > 1.0 && statorCurrent > 9.0;
  }

  public boolean checkMuscleUpCurrent() {
    double supplyCurrent = Math.abs(muscleUpMotor.getSupplyCurrent().getValue().magnitude());
    double statorCurrent = Math.abs(muscleUpMotor.getStatorCurrent().getValue().magnitude());

    return supplyCurrent > 0.2 && statorCurrent > 6.0;
  }

  public void resetPullUpCurrentLimits() {
    pullUpMotorR.updateCurrentLimits(
        Constants.Climber.DEFAULT_SUPPLY_CURRENT, Constants.Climber.DEFAULT_STATOR_CURRENT);
    pullUpMotorL.updateCurrentLimits(
        Constants.Climber.DEFAULT_SUPPLY_CURRENT, Constants.Climber.DEFAULT_STATOR_CURRENT);
  }

  public void resetMuscleUpCurrentLimits() {
    muscleUpMotor.updateCurrentLimits(
        Constants.Climber.DEFAULT_STATOR_CURRENT, Constants.Climber.DEFAULT_SUPPLY_CURRENT);
  }

  public void resetPullUpPositionToZero() {
    pullUpMotorR.setPosition(0);
  }

  public void resetPullUpPositionToTop() {
    pullUpMotorR.setPosition(
        Constants.Climber.PullUp.PULL_DOWN_POS_METERS
            * Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS);
  }

  public Command resetPullUpPositionToZeroCommand() {
    return runOnce(() -> this.resetPullUpPositionToZero());
  }

  public void resetMuscleUpPositionToZero() {
    muscleUpMotor.setPosition(0);
  }

  // Comands
  public Command brakeCommand() {
    return runOnce(this::brakeClimb);
  }

  public Command brakeWithoutServoCommand() {
    return runOnce(this::stopClimbWithoutBrake);
  }

  public Command MuscleUpCommand(double angle) {
    return runOnce(() -> setMuscleUpPosition(angle)).until(this::isMuscleUpAtPosition);
  }

  public Command PullUpCommand(double position) {
    return runOnce(() -> setPullUpPosition(position)).until(this::isPullUpAtPosition);
  }

  public Command SitUpCommand(double angle) {
    return run(() -> setSitUpPosition(angle)); // .until(this::isSitUpAtPosition);
  }

  // separate command groups to incorporate driveToPose

  public Command L1ClimbCommand() {
    return Commands.sequence(
        PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        brakeCommand());
  }

  public Command L2ClimbCommand() {
    return Commands.sequence(
        // rest of L1 climb
        MuscleUpCommand(Constants.Climber.MuscleUp.L1_MUSCLE_UP_FORWARD),
        SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE_DEGREES),
        // L2 Climb
        PullUpCommand(Constants.Climber.PullUp.L2_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS));
  }

  public Command L3ClimbCommand() {
    return Commands.sequence(
        // L2 climb & going up to the rung
        L2ClimbCommand(),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.L1_MUSCLE_UP_FORWARD),
        // L3 climb
        PullUpCommand(Constants.Climber.PullUp.L3_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.L3_MUSCLE_UP_FORWARD),
        SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE_DEGREES));
  }

  public Command abortCommand() {
    return runOnce(this::brakeClimb);
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Subsystems/Climber/SitUpPositionDeg", sitUpMotor.getPosition().getValueAsDouble() * 360f);
    DogLog.log(
        "Subsystems/Climber/MuscleUpPositionDeg",
        muscleUpMotor.getPosition().getValueAsDouble()
            * Constants.Climber.MuscleUp.ARM_DEGREES_PER_MOTOR_ROTS);
    DogLog.log(
        "Subsystems/Climber/PullUpPositionMeter",
        pullUpMotorR.getPosition().getValueAsDouble()
            / Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS);

    DogLog.log(
        "Subsystems/Climber/SitUpPositionFromEncoderRots", getSitUpPosInRotationsFromEncoder());

    DogLog.log(
        "Subsystems/Climber/CurrentLimits/MuscleUpStator",
        muscleUpMotor.getStatorCurrent().getValue().magnitude());
    DogLog.log(
        "Subsystems/Climber/CurrentLimits/MuscleUpSupply",
        muscleUpMotor.getSupplyCurrent().getValue().magnitude());
    DogLog.log(
        "Subsystems/Climber/CurrentLimits/PullUpStator",
        pullUpMotorR.getStatorCurrent().getValue().magnitude());
    DogLog.log(
        "Subsystems/Climber/CurrentLimits/PullUpSupply",
        pullUpMotorR.getSupplyCurrent().getValue().magnitude());
  }
}
