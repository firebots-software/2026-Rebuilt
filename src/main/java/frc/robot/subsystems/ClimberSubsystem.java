package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;

  private static double muscleUpTolerance = .1;
  private static double sitUpTolerance = .1;
  private static double pullUpTolerance = .1;

  private final LoggedTalonFX muscleUpMotor, sitUpMotor, pullUpMotorR, pullUpMotorL;
  private double sitUpTargetDeg, muscleUpTargetDeg, pullUpTargetPosition;
  private final DutyCycleEncoder muscleUpEncoder, sitUpEncoder;

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  public ClimberSubsystem() {
    CurrentLimitsConfigs regClc =
        new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);

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

    muscleUpMotor.getConfigurator().apply(s0c);
    sitUpMotor.getConfigurator().apply(s0c);
    pullUpMotorR.getConfigurator().apply(s0c);
    pullUpMotorL.getConfigurator().apply(s0c);

    muscleUpMotor.getConfigurator().apply(regClc);
    sitUpMotor.getConfigurator().apply(specialClc);
    pullUpMotorR.getConfigurator().apply(regClc);
    pullUpMotorL.getConfigurator().apply(regClc);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    muscleUpMotor.getConfigurator().apply(moc);
    sitUpMotor.getConfigurator().apply(moc);
    pullUpMotorR.getConfigurator().apply(moc);
    pullUpMotorL.getConfigurator().apply(moc);

    MotionMagicConfigs mmc =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Climber.mmcV)
            .withMotionMagicAcceleration(Constants.Climber.mmcA);

    muscleUpMotor.getConfigurator().apply(mmc);
    sitUpMotor.getConfigurator().apply(mmc);
    pullUpMotorR.getConfigurator().apply(mmc);
    pullUpMotorL.getConfigurator().apply(mmc);

    sitUpEncoder = new DutyCycleEncoder(Constants.Climber.SitUp.ENCODER_PORT);
    muscleUpEncoder = new DutyCycleEncoder(Constants.Climber.MuscleUp.ENCODER_PORT);
  }

  public void setSitUpPosition(double degrees) {
    sitUpTargetDeg = degrees / Constants.Climber.SitUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT;
    sitUpMotor.setControl(new MotionMagicVoltage(sitUpTargetDeg));
  }

  public void setMuscleUpPosition(double degrees) {
    muscleUpTargetDeg = degrees / Constants.Climber.MuscleUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT;
    muscleUpMotor.setControl(new MotionMagicVoltage(muscleUpTargetDeg));
  }

  public void setPullUpPosition(double metersFromZero) {
    pullUpTargetPosition =
        metersFromZero / Constants.Climber.PullUp.MOTOR_ROTS_TO_METERS_OF_BELT_TRAVERSAL;
    pullUpMotorR.setControl(new MotionMagicVoltage(pullUpTargetPosition));
  }

  public boolean isSitUpAtPosition() {
    return Math.abs(
            sitUpMotor.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT
                - sitUpTargetDeg)
        <= sitUpTolerance;
  }

  public boolean isMuscleUpAtPosition() {
    return Math.abs(
            muscleUpMotor.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT
                - muscleUpTargetDeg)
        <= muscleUpTolerance;
  }

  public boolean isPullUpAtPosition() {
    return Math.abs(
            pullUpMotorR.getPosition().getValueAsDouble()
                    * Constants.Climber.SitUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT
                - pullUpTargetPosition)
        <= pullUpTolerance;
  }

  public double getMuscleUpPosInRotationsFromEncoder() {
    return muscleUpEncoder.get() * Constants.Climber.MuscleUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS;
  }

  public double getSitUpPosInRotationsFromEncoder() {
    return sitUpEncoder.get() * Constants.Climber.SitUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS;
  }

  @Override
  public void periodic() {
    DogLog.log(
        "Climber/SitUpPositionDeg",
        sitUpMotor.getPosition().getValueAsDouble()
            * Constants.Climber.SitUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT);
    DogLog.log(
        "Climber/MuscleUpPositionDeg",
        muscleUpMotor.getPosition().getValueAsDouble()
            * Constants.Climber.MuscleUp.MOTOR_ROTS_TO_DEGREES_OF_ARM_ROT);
    DogLog.log(
        "Climber/PullUpPositionMeter",
        pullUpMotorR.getPosition().getValueAsDouble()
            * Constants.Climber.PullUp.MOTOR_ROTS_TO_METERS_OF_BELT_TRAVERSAL);

    DogLog.log("Climber/SitUpPositionFromEncoderRots", getSitUpPosInRotationsFromEncoder());
    DogLog.log("Climber/MuscleUpPositionFromEncoderRots", getMuscleUpPosInRotationsFromEncoder());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
