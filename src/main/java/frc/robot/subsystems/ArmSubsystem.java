package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;
  private LoggedTalonFX topLeft, topRight, bottomLeft, bottomRight, master;
  private MotionMagicConfigs mmc;
  private DutyCycleEncoder encoder;
  private double targetDegrees;

  private double armOffset;

  public ArmSubsystem() {
    // initialize motors
    topLeft = new LoggedTalonFX(Constants.Arm.topLeftMotor.port, Constants.Arm.canbus);
    topRight = new LoggedTalonFX(Constants.Arm.topRightMotor.port, Constants.Arm.canbus);
    bottomLeft = new LoggedTalonFX(Constants.Arm.bottomLeftMotor.port, Constants.Arm.canbus);
    bottomRight = new LoggedTalonFX(Constants.Arm.bottomRightMotor.port, Constants.Arm.canbus);

    CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40);
    Slot0Configs s0c = new Slot0Configs()
        .withKP(Constants.Arm.armKP)
        .withKI(Constants.Arm.armKI)
        .withKD(Constants.Arm.armKD);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    // define master/followers
    master = topLeft;
    Follower follower = new Follower(Constants.Arm.topLeftMotor.port, MotorAlignmentValue.Aligned);
    // bottom left motor needs to be inverted
    Follower invertedFollower = new Follower(Constants.Arm.topLeftMotor.port, MotorAlignmentValue.Opposed);
    topRight.setControl(invertedFollower);
    bottomRight.setControl(invertedFollower);
    bottomLeft.setControl(follower);

    TalonFXConfigurator masterConfig = master.getConfigurator();
    TalonFXConfigurator topRightConfig = topRight.getConfigurator();
    TalonFXConfigurator bottomLeftConfig = bottomLeft.getConfigurator();
    TalonFXConfigurator bottomRightConfig = bottomRight.getConfigurator();

    masterConfig.apply(moc);
    topRightConfig.apply(moc);
    bottomLeftConfig.apply(moc);
    bottomRightConfig.apply(moc);

    masterConfig.apply(clc);
    topRightConfig.apply(clc);
    bottomLeftConfig.apply(clc);
    bottomRightConfig.apply(clc);

    masterConfig.apply(s0c);
    topRightConfig.apply(s0c);
    bottomLeftConfig.apply(s0c);
    bottomRightConfig.apply(s0c);

    mmc = new MotionMagicConfigs();
    // change values as needed
    mmc.MotionMagicCruiseVelocity = 50;
    mmc.MotionMagicAcceleration = 50;
    masterConfig.apply(mmc);

    encoder = new DutyCycleEncoder(0);
    targetDegrees = 4.0; // intake angle

    // set arm offset
    armOffset = getDegrees();
    resetPosition();
  }

  public static ArmSubsystem getInstance() {
    if (instance == null)
      instance = new ArmSubsystem();
    return instance;
  }

  public double calculateRots(double degrees) {
    return ((degrees / 360d) * Constants.Arm.conversionFactor) + Constants.Arm.absoluteHorizontalOffset + armOffset;
  }

  public void setPosition(double degrees) {
    master.setControl(new MotionMagicVoltage(calculateRots(MathUtil.clamp(degrees, 3, 110))));
  }

  public void setTarget(double degrees) {
    targetDegrees = degrees;
  }

  public void setPosToTarget() {
    setPosition(targetDegrees);
  }

  private double getDegrees() {
    // use arm pos in rotations to get degrees
    return (master.getPosition().getValueAsDouble() / Constants.Arm.conversionFactor) * 360d;
  }

  private double getAbsolutePosition() {
    return (encoder.get()
        - Constants.Arm.absoluteEncoderHorizontal
        + Constants.Arm.absoluteHorizontalOffset
        + 1d)
        % 1;
  }

  public void resetPosition() {
    master.setPosition(getAbsolutePosition() * Constants.Arm.conversionFactor);
  }

  public void resetOffset() {
    armOffset = getDegrees();
  }

  @Override
  public void periodic() {
    DogLog.log("DogLog/Arm/targetAngle", targetDegrees + armOffset);
    DogLog.log("DogLog/Arm/masterVelocity", master.getVelocity().getValueAsDouble());
    DogLog.log("DogLog/Arm/masterPosition", master.getPosition().getValueAsDouble());
    DogLog.log("DogLog/Arm/armDegrees", getDegrees());
    DogLog.log("DogLog/Arm/armDegreesCorrected", getDegrees() - armOffset);
    DogLog.log("DogLog/Arm/armDegreesAbsolute", getAbsolutePosition());
    DogLog.log("DogLog/Arm/armOffset", armOffset);
    DogLog.log("DogLog/Arm/rawEncoderValue", encoder.get());
  }
}
