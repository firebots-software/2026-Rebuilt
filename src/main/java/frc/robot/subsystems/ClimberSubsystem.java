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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
  private final CANcoder muscleUpEncoder, sitUpEncoder;
  private final Servo brake;

  private final VelocityVoltage m_veclocityRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0);

  public ClimberSubsystem() {
    CurrentLimitsConfigs regClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.DEFAULT_STATOR_CURRENT)
            .withSupplyCurrentLimit(Constants.Climber.DEFAULT_SUPPLY_CURRENT);

    CurrentLimitsConfigs specialClc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Climber.SitUp.CURRENT_STATOR_LIMIT)
            .withSupplyCurrentLimit(Constants.Climber.SitUp.CURRENT_SUPPLY_LIMIT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Climber.KP)
            .withKI(Constants.Climber.KI)
            .withKD(Constants.Climber.KD);

    muscleUpMotor = new LoggedTalonFX(Constants.Climber.MuscleUp.MOTOR_PORT);

    sitUpMotor = new LoggedTalonFX(Constants.Climber.SitUp.MOTOR_PORT);

    pullUpMotorR = new LoggedTalonFX(Constants.Climber.PullUp.MOTOR_PORT_R);
    pullUpMotorL = new LoggedTalonFX(Constants.Climber.PullUp.MOTOR_PORT_L);
    pullUpMotorL.setControl(new Follower(pullUpMotorR.getDeviceID(), MotorAlignmentValue.Opposed));

    brake = new Servo(Constants.Climber.BRAKE_PORT);

    TalonFXConfigurator muscleUpConfigurator = muscleUpMotor.getConfigurator();
    TalonFXConfigurator sitUpConfigurator = sitUpMotor.getConfigurator();
    TalonFXConfigurator pullUpLeftConfigurator = pullUpMotorL.getConfigurator();
    TalonFXConfigurator pullUpRightConfigurator = pullUpMotorR.getConfigurator();

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Climber.mmcV)
            .withMotionMagicAcceleration(Constants.Climber.mmcA);

    muscleUpConfigurator.apply(s0c);
    sitUpConfigurator.apply(s0c);
    pullUpRightConfigurator.apply(s0c);
    pullUpLeftConfigurator.apply(s0c);

    muscleUpConfigurator.apply(regClc);
    sitUpConfigurator.apply(specialClc);
    pullUpRightConfigurator.apply(regClc);
    pullUpLeftConfigurator.apply(regClc);

    muscleUpConfigurator.apply(moc);
    sitUpConfigurator.apply(moc);
    pullUpRightConfigurator.apply(moc);
    pullUpLeftConfigurator.apply(moc);

    muscleUpConfigurator.apply(mmc);
    sitUpConfigurator.apply(mmc);
    pullUpRightConfigurator.apply(mmc);
    pullUpLeftConfigurator.apply(mmc);

    // create fusedcancoders
    sitUpEncoder = new CANcoder(Constants.Climber.SitUp.ENCODER_PORT);
    muscleUpEncoder = new CANcoder(Constants.Climber.MuscleUp.ENCODER_PORT);

    MagnetSensorConfigs canCoderConfig =
        new CANcoderConfiguration()
            .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    sitUpEncoder
        .getConfigurator()
        .apply(
            canCoderConfig.withMagnetOffset(Rotations.of(Constants.Climber.SitUp.ENCODER_OFFSET)));

    muscleUpEncoder
        .getConfigurator()
        .apply(
            canCoderConfig.withMagnetOffset(
                Rotations.of(Constants.Climber.MuscleUp.ENCODER_OFFSET)));

    sitUpConfigurator.apply(
        new TalonFXConfiguration()
            .Feedback.withFeedbackRemoteSensorID(sitUpEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(
                    Constants.Climber.SitUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS)
                .withRotorToSensorRatio(Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT));

    muscleUpConfigurator.apply(
        new TalonFXConfiguration()
            .Feedback.withFeedbackRemoteSensorID(muscleUpEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(
                    Constants.Climber.MuscleUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS)
                .withRotorToSensorRatio(
                    Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT));
  }

  public void setSitUpPosition(double degrees) {
    sitUpTargetDeg = degrees / Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;
    sitUpMotor.setControl(m_motionMagicRequest.withPosition(sitUpTargetDeg));
  }

  public void setMuscleUpPosition(double degrees) {
    muscleUpTargetDeg = degrees / Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;
    muscleUpMotor.setControl(m_motionMagicRequest.withPosition(muscleUpTargetDeg));
  }

  public void setPullUpPosition(double metersFromZero) {
    pullUpTargetPosition =
        metersFromZero / Constants.Climber.PullUp.MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL;
    pullUpMotorR.setControl(m_motionMagicRequest.withPosition(pullUpTargetPosition));
  }

  public boolean isSitUpAtPosition() {
    return Math.abs(
            sitUpMotor.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT
                - sitUpTargetDeg)
        <= Constants.Climber.SitUp.SIT_UP_TOLERANCE;
  }

  public boolean isMuscleUpAtPosition() {
    return Math.abs(
            muscleUpMotor.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT
                - muscleUpTargetDeg)
        <= Constants.Climber.MuscleUp.MUSCLE_UP_TOLERANCE;
  }

  public boolean isPullUpAtPosition() {
    return Math.abs(
            pullUpMotorR.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT
                - pullUpTargetPosition)
        <= Constants.Climber.PullUp.PULL_UP_TOLERANCE;
  }

  public double getMuscleUpPosInRotationsFromEncoder() {
    return muscleUpEncoder.getAbsolutePosition().getValueAsDouble()
        * Constants.Climber.MuscleUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS;
  }

  public double getSitUpPosInRotationsFromEncoder() {
    return sitUpEncoder.getAbsolutePosition().getValueAsDouble()
        * Constants.Climber.SitUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS;
  }

  public void stopSitUp() {
    sitUpTargetDeg = getSitUpPosInRotationsFromEncoder();
    sitUpMotor.setPosition(sitUpTargetDeg);
  }

  public void stopPullUp() {
    pullUpTargetPosition = pullUpMotorR.getPosition().getValueAsDouble();
    pullUpMotorR.setPosition(pullUpTargetPosition);
  }

  public void stopMuscleUp() {
    muscleUpTargetDeg = getMuscleUpPosInRotationsFromEncoder();
    muscleUpMotor.setPosition(muscleUpTargetDeg);
  }

  public void brakeClimb() {
    brake.setAngle(Constants.Climber.BRAKE_ANGLE);
    stopSitUp();
    stopPullUp();
    stopMuscleUp();
  }

  // Zeroing climb functions (only pull up because it doesn't have an encoder):

  public void reduceCurrentLimits() {
    pullUpMotorR.updateCurrentLimits(30, 10);
    pullUpMotorL.updateCurrentLimits(30, 10);
  }

  public void movePullUpDown() {
    pullUpMotorR.setControl(m_veclocityRequest.withVelocity(-5));
  }

  public boolean checkCurrent() {
    double supplyCurrent = Math.abs(pullUpMotorR.getSupplyCurrent().getValue().magnitude());
    double statorCurrent = Math.abs(pullUpMotorR.getStatorCurrent().getValue().magnitude());

    return supplyCurrent > 1.0 && statorCurrent > 20.0;
  }

  public void resetCurrentLimits() {
    pullUpMotorR.updateCurrentLimits(
        Constants.Climber.DEFAULT_SUPPLY_CURRENT, Constants.Climber.DEFAULT_STATOR_CURRENT);
    pullUpMotorL.updateCurrentLimits(
        Constants.Climber.DEFAULT_SUPPLY_CURRENT, Constants.Climber.DEFAULT_STATOR_CURRENT);
  }

  public void resetPullUpPositionToZero() {
    pullUpMotorR.setPosition(0);
  }

  // Comands
  public Command brakeCommand() {
    return runOnce(this::brakeClimb);
  }

  public Command MuscleUpCommand(double angle) {
    return runOnce(() -> setMuscleUpPosition(angle)).until(() -> isMuscleUpAtPosition());
  }

  public Command PullUpCommand(double position) {
    return runOnce(() -> setPullUpPosition(position)).until(this::isPullUpAtPosition);
  }

  public Command SitUpCommand(double angle) {
    return runOnce(() -> setSitUpPosition(angle)).until(this::isSitUpAtPosition);
  }

  // separate command groups to incorporate driveToPose

  public Command L1ClimbCommand() {
    return Commands.sequence(
        PullUpCommand(Constants.Climber.PullUp.L1_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        brakeCommand());
  }

  public Command L2ClimbCommand() {
    return Commands.sequence(
        // rest of L1 climb
        MuscleUpCommand(Constants.Climber.MuscleUp.L1_MUSCLE_UP_FORWARD),
        SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE),
        // L2 Climb
        PullUpCommand(Constants.Climber.PullUp.L2_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.L2_MUSCLE_UP_FORWARD),
        SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE));
  }

  public Command L3ClimbCommand() {
    return Commands.sequence(
        L2ClimbCommand(),
        // L3 climb
        PullUpCommand(Constants.Climber.PullUp.L3_REACH_POS),
        SitUpCommand(Constants.Climber.SitUp.SIT_UP_ANGLE),
        MuscleUpCommand(Constants.Climber.MuscleUp.MUSCLE_UP_BACK),
        PullUpCommand(Constants.Climber.PullUp.PULL_DOWN_POS),
        MuscleUpCommand(Constants.Climber.MuscleUp.L3_MUSCLE_UP_FORWARD),
        SitUpCommand(Constants.Climber.SitUp.SIT_BACK_ANGLE));
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Climber/SitUpPositionDeg",
        sitUpMotor.getPosition().getValueAsDouble()
            * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT);
    DogLog.log(
        "Climber/MuscleUpPositionDeg",
        muscleUpMotor.getPosition().getValueAsDouble()
            * Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT);
    DogLog.log(
        "Climber/PullUpPositionMeter",
        pullUpMotorR.getPosition().getValueAsDouble()
            * Constants.Climber.PullUp.MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL);

    DogLog.log("Climber/SitUpPositionFromEncoderRots", getSitUpPosInRotationsFromEncoder());
    DogLog.log("Climber/MuscleUpPositionFromEncoderRots", getMuscleUpPosInRotationsFromEncoder());
  }
}
