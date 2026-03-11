package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ClimberSubsystem extends SubsystemBase {
  private final LoggedTalonFX muscleUpMotor, sitUpMotor, pullUpMotorR, pullUpMotorL;
  private double sitUpTargetDeg, muscleUpTargetDeg, pullUpTargetPositionMeters;
  private final CANcoder sitUpEncoder;
  private final Servo brake;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0.0);
  private final PositionVoltage m_positionRequest = new PositionVoltage(0);

  public ClimberSubsystem() {
    CurrentLimitsConfigs idleClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.DEFAULT_STATOR_CURRENT)
            .withSupplyCurrentLimit(Constants.Climber.DEFAULT_SUPPLY_CURRENT);

    CurrentLimitsConfigs sitUpClimbingClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.SitUp.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Climber.SitUp.SUPPLY_CURRENT_LIMIT);

    CurrentLimitsConfigs pullUpClimbingClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.PullUp.CLIMBING_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Climber.PullUp.CLIMBING_SUPPLY_CURRENT_LIMIT);

    // CurrentLimitsConfigs muscleUpClimbingClc =
    //     new CurrentLimitsConfigs()
    //         .withStatorCurrentLimit(Constants.Climber.MuscleUp.CLIMBING_STATOR_CURRENT_LIMIT)
    //         .withSupplyCurrentLimit(Constants.Climber.MuscleUp.CLIMBING_STATOR_CURRENT_LIMIT);

    Slot0Configs sitUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.SitUp.KP)
            .withKI(Constants.Climber.SitUp.KI)
            .withKD(Constants.Climber.SitUp.KD);
    Slot0Configs pullUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.PullUp.KP)
            .withKI(Constants.Climber.PullUp.KI)
            .withKD(Constants.Climber.PullUp.KD)
            .withKV(Constants.Climber.PullUp.KV)
            .withKG(Constants.Climber.PullUp.KG)
            .withKS(Constants.Climber.PullUp.KS);
    Slot0Configs muscleUps0c =
        new Slot0Configs()
            .withKP(Constants.Climber.MuscleUp.KP)
            .withKI(Constants.Climber.MuscleUp.KI)
            .withKD(Constants.Climber.MuscleUp.KD);

    MotorOutputConfigs mocCCWPos =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    MotorOutputConfigs mocCWPos =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

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

    sitUpEncoder =
      new CANcoder(
        Constants.Climber.SitUp.ENCODER_PORT, Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
      
    MagnetSensorConfigs canCoderConfig =
      new CANcoderConfiguration()
        .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(Rotations.of(Constants.Climber.SitUp.ENCODER_OFFSET));
    
    sitUpEncoder.getConfigurator().apply(canCoderConfig);
    
    TalonFXConfiguration muscleUpConfig =
      new TalonFXConfiguration()
        .withSlot0(muscleUps0c)
        .withCurrentLimits(idleClc)
        .withMotorOutput(mocCWPos);
    
    FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(sitUpEncoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(Constants.Climber.SitUp.ENCODER_ROTS_PER_ARM_ROT)
        .withRotorToSensorRatio(Constants.Climber.SitUp.MOTOR_ROTS_PER_ENCODER_ROT);

    TalonFXConfiguration sitUpConfig =
      new TalonFXConfiguration()
        .withSlot0(sitUps0c)
        .withCurrentLimits(sitUpClimbingClc)
        .withMotorOutput(mocCWPos)
        .withFeedback(feedbackConfigs);
    
    TalonFXConfiguration pullUpLeftConfig =
      new TalonFXConfiguration()
        .withSlot0(pullUps0c)
        .withCurrentLimits(pullUpClimbingClc)
        .withMotorOutput(mocCCWPos);
    
    TalonFXConfiguration pullUpRightConfig =
      new TalonFXConfiguration()
        .withSlot0(pullUps0c)
        .withCurrentLimits(pullUpClimbingClc)
        .withMotorOutput(mocCWPos);
    
    muscleUpMotor.getConfigurator().apply(muscleUpConfig);
    sitUpMotor.getConfigurator().apply(sitUpConfig);
    pullUpMotorL.getConfigurator().apply(pullUpLeftConfig);
    pullUpMotorR.getConfigurator().apply(pullUpRightConfig);
    
    brake = new Servo(Constants.Climber.BRAKE_PORT);
  }

  public void setSitUpPosition(double degrees) {
    sitUpTargetDeg = degrees;
    sitUpMotor.setControl(m_positionRequest.withPosition(sitUpTargetDeg / 360.0));
  }

  public void setMuscleUpPosition(double degrees) {
    muscleUpTargetDeg = degrees;
    muscleUpMotor.setControl(
        m_positionRequest.withPosition(
            muscleUpTargetDeg * Constants.Climber.MuscleUp.MOTOR_ROTS_PER_ARM_DEGREES));
  }

  public void setPullUpPosition(double metersFromZero) {
    pullUpTargetPositionMeters = metersFromZero;
    pullUpMotorR.setControl(
        m_positionRequest.withPosition(
            pullUpTargetPositionMeters * Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS));
  }

  public boolean isSitUpAtPosition() {
    return Math.abs((sitUpMotor.getCachedPositionRotations() * 360.0) - sitUpTargetDeg)
        <= Constants.Climber.SitUp.SIT_UP_TOLERANCE;
  }

  public boolean isMuscleUpAtPosition() {
    return Math.abs(
            muscleUpMotor.getCachedPositionRotations()
                    * Constants.Climber.MuscleUp.ARM_DEGREES_PER_MOTOR_ROTS
                - muscleUpTargetDeg)
        <= Constants.Climber.MuscleUp.MUSCLE_UP_TOLERANCE;
  }

  public boolean isPullUpAtPosition() {
    return Math.abs(
            (pullUpMotorR.getCachedPositionRotations()
                    / Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS)
                - pullUpTargetPositionMeters)
        <= Constants.Climber.PullUp.PULL_UP_TOLERANCE_METERS;
  }

  public double getSitUpCancoderPositionRaw() {
    return sitUpEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public Rotation2d getSitUpUnfusedPosition() {
    return new Rotation2d(
        Units.rotationsToRadians(
            getSitUpCancoderPositionRaw() * Constants.Intake.Arm.ARM_ROTS_PER_CANCODER_ROT));
  }

  public Rotation2d getSitUpPosition() {
    return new Rotation2d(Units.rotationsToRadians(sitUpMotor.getCachedPositionRotations()));
  }

  public void stopSitUp() {
    sitUpTargetDeg = sitUpMotor.getCachedPositionRotations();
    setSitUpPosition(sitUpTargetDeg);
  }

  public void stopPullUp() {
    pullUpTargetPositionMeters =
        pullUpMotorR.getCachedPositionRotations()
            / Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS;
    setPullUpPosition(pullUpTargetPositionMeters);
  }

  public void stopMuscleUp() {
    muscleUpTargetDeg =
        muscleUpMotor.getCachedPositionRotations()
            * Constants.Climber.MuscleUp.ARM_DEGREES_PER_MOTOR_ROTS;
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
    // stopSitUp();
    stopPullUp();
    stopMuscleUp();
  }

  // Zeroing climb functions (only pull up because it doesn't have an encoder):
  public void reducePullUpCurrentLimits() {
    pullUpMotorR.updateCurrentLimits(
        Constants.Climber.PullUp.ZEROING_STATOR_CURRENT_LIMIT,
        Constants.Climber.PullUp.ZEROING_SUPPLY_CURRENT_LIMIT);
    pullUpMotorL.updateCurrentLimits(
        Constants.Climber.PullUp.ZEROING_STATOR_CURRENT_LIMIT,
        Constants.Climber.PullUp.ZEROING_SUPPLY_CURRENT_LIMIT);
  }

  public void reduceMuscleUpCurrentLimits() {
    muscleUpMotor.updateCurrentLimits(
        Constants.Climber.MuscleUp.ZEROING_STATOR_CURRENT_LIMIT,
        Constants.Climber.MuscleUp.ZEROING_SUPPLY_CURRENT_LIMIT);
  }

  public void movePullUpDownVelocity() {
    pullUpMotorR.setControl(
        m_velocityRequest.withVelocity(Constants.Climber.PullUp.PULL_DOWN_VELOCITY));
  }

  public void movePullUpUpVelocity() {
    pullUpMotorR.setControl(m_velocityRequest.withVelocity(3.0));
  }

  public void movePullUpUpWithVoltage() {
    pullUpMotorR.setControl(m_voltageRequest.withOutput(4.0));
  }

  public void movePullUpDownWithVoltage() {
    pullUpMotorR.setControl(m_voltageRequest.withOutput(-4.0));
  }

  public Command movePullUpUpWithVoltageCommand() {
    return startEnd(this::movePullUpUpWithVoltage, this::stopPullUp);
  }

  public Command movePullUpDownWithVoltageCommand() {
    return startEnd(this::movePullUpDownWithVoltage, this::stopPullUp);
  }

  public void moveMuscleUpInWithVoltage() {
    // muscleUpMotor.setControl(
    // m_velocityRequest.withVelocity(Constants.Climber.MuscleUp.MUSCLEUP_DOWN_VELOCITY));
    muscleUpMotor.setControl(m_voltageRequest.withOutput(-3.0));
  }

  public void moveMuscleUpOutWithVoltage() {
    // muscleUpMotor.setControl(
    // m_velocityRequest.withVelocity(Constants.Climber.MuscleUp.MUSCLEUP_DOWN_VELOCITY));
    muscleUpMotor.setControl(m_voltageRequest.withOutput(3.0));
  }

  public Command moveMuscleUpOutCommand() {
    return startEnd(this::moveMuscleUpOutWithVoltage, this::stopMuscleUp);
  }

  public Command moveMuscleUpInCommand() {
    return startEnd(this::moveMuscleUpInWithVoltage, this::stopMuscleUp);
  }

  public boolean checkPullUpCurrent() {
    double supplyCurrent = Math.abs(pullUpMotorR.getSupplyCurrent().getValue().magnitude());
    double statorCurrent = Math.abs(pullUpMotorR.getStatorCurrent().getValue().magnitude());

    return supplyCurrent > 0.48 && statorCurrent > 13.8;
  }

  public boolean checkMuscleUpCurrent() {
    double supplyCurrent = Math.abs(muscleUpMotor.getSupplyCurrent().getValue().magnitude());
    double statorCurrent = Math.abs(muscleUpMotor.getStatorCurrent().getValue().magnitude());

    return supplyCurrent > 0.2 && statorCurrent > 5.0;
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
    pullUpMotorL.setPosition(0);
  }

  public void resetPullUpPositionToTop() {
    pullUpMotorR.setPosition(
        Constants.Climber.PullUp.PULL_DOWN_POS_METERS
            * Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS);
    pullUpMotorL.setPosition(
        Constants.Climber.PullUp.PULL_DOWN_POS_METERS
            * Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS);
  }

  public Command resetPullUpPositionToZeroCommand() {
    return runOnce(this::resetPullUpPositionToZero);
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
    return run(() -> setMuscleUpPosition(angle)).until(this::isMuscleUpAtPosition);
  }

  public Command pullUpCommand(double position) {
    return run(() -> setPullUpPosition(position)).until(this::isPullUpAtPosition);
  }

  public Command PullUpToCertainPositionCommand(double positionMeters) {
    return runOnce(() -> this.setPullUpPosition(positionMeters));
  }

  public Command sitUpCommand(double angle) {
    return run(() -> setSitUpPosition(angle)); // .until(this::isSitUpAtPosition);
  }

  public Command SitUpCertainPos(double angle) {
    return runOnce(() -> setSitUpPosition(angle));
  }

  // separate command groups to incorporate driveToPose
  public Command L1ClimbCommand() {
    return Commands.sequence(
        pullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        sitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        pullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        brakeCommand());
  }

  public Command l2ClimbCommand() {
    return Commands.sequence(
        // rest of L1 climb
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_FORWARD),
        sitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE_DEGREES),
        // L2 Climb
        pullUpCommand(Constants.Climber.PullUp.L2_REACH_POS),
        sitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        pullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS));
  }

  public Command l3ClimbCommand() {
    return Commands.sequence(
        // L2 climb & going up to the rung
        l2ClimbCommand(),
        pullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_FORWARD),
        // L3 climb
        pullUpCommand(Constants.Climber.PullUp.L3_REACH_POS),
        sitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE_DEGREES),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        pullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_FORWARD),
        sitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE_DEGREES));
  }

  public Command abortCommand() {
    return runOnce(this::brakeClimb);
  }

  public Command sitUpVoltageCommand(double voltage) {
    return runOnce(() -> sitUpMotor.setControl(new VoltageOut(voltage)));
  }

  @Override
  public void periodic() {
    DogLog.log("Subsystems/Climber/SitUpPositionDeg", getSitUpPosition().getDegrees());
    DogLog.log(
        "Subsystems/Climber/SitUpPositionDegUnfused", getSitUpUnfusedPosition().getDegrees());
    DogLog.log(
        "Subsystems/Climber/SitUpPositioDegRawFromCancoder", getSitUpCancoderPositionRaw() * 360.0);
    DogLog.log(
        "Subsystems/Climber/MuscleUpPositionDeg",
        muscleUpMotor.getCachedPositionRotations()
            * Constants.Climber.MuscleUp.ARM_DEGREES_PER_MOTOR_ROTS);
    DogLog.log(
        "Subsystems/Climber/PullUpPositionMeter",
        pullUpMotorR.getCachedPositionRotations()
            / Constants.Climber.PullUp.MOTOR_ROTS_PER_BELT_METERS);
    DogLog.log("Subsystems/Climber/SitUpPositionFromEncoderRots", getSitUpCancoderPositionRaw());
    // DogLog.log(
    //     "Subsystems/Climber/CurrentLimits/MuscleUpStator",
    //     Math.abs(muscleUpMotor.getCachedStatorCurrentA()));
    // DogLog.log(
    //     "Subsystems/Climber/CurrentLimits/MuscleUpSupply",
    //     Math.abs(muscleUpMotor.getCachedSupplyCurrentA()));
    // DogLog.log(
    //     "Subsystems/Climber/CurrentLimits/PullUpStator",
    //     Math.abs(pullUpMotorR.getCachedStatorCurrentA()));
    // DogLog.log(
    //     "Subsystems/Climber/CurrentLimits/PullUpSupply",
    //     Math.abs(pullUpMotorR.getCachedSupplyCurrentA()));
  }
}
