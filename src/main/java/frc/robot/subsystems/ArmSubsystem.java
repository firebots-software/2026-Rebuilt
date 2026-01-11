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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance;

  // variables storing the four motors that are controlling the robot arm
  private LoggedTalonFX rightTopMotor, rightBottomMotor, leftTopMotor, leftBottomMotor;

  // variable that you refer to the "master" motor with
  private LoggedTalonFX master;

  // absolute encoder that tells us how far the arm has travelled 
  private DutyCycleEncoder revEncoder;
  private boolean enableArm;

  // Motion Magic Configurations relating to the arm motors
  private MotionMagicConfigs mmc;

  private boolean initialized = false;

  // target angle of the arm in degrees
  private double targetDegrees;

  
  private double armHorizontalOffset;

  public ArmSubsystem() {
    // Current Limits established for the arm
    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Constants.Arm.ARM_STATOR_CURRENT_LIMIT_AMPS);

    // Review: Brake mode refers to the fact that when the robot is powered on but the motor output is 0, make the motor
    // resistant to moving ( this is why when the robot is powered on and you drop the arm it slowly goes down instead of slamming
    // immediately )
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    // TODO: Configure PID and Feedforward values of the Arm -> refer to the 'TODO' in Constants.java 
    Slot0Configs s0c = new Slot0Configs().withKP(Constants.Arm.S0C_KP).withKI(Constants.Arm.S0C_KI).withKD(Constants.Arm.S0C_KD).withKV(0.12).withKG(Constants.Arm.S0C_KG).withKS(Constants.Arm.S0C_KS).withKA(Constants.Arm.S0C_KA);

    // Initialize motors
    rightTopMotor = new LoggedTalonFX("ArmRightTop", Constants.Arm.RT_PORT, Constants.Arm.CANBUS_NAME);
    rightBottomMotor = new LoggedTalonFX("ArmRightBottom", Constants.Arm.RB_PORT, Constants.Arm.CANBUS_NAME);
    leftTopMotor = new LoggedTalonFX("ArmLefttop", Constants.Arm.LT_PORT, Constants.Arm.CANBUS_NAME);
    leftBottomMotor = new LoggedTalonFX("ArmLeftBottom", Constants.Arm.LB_PORT, Constants.Arm.CANBUS_NAME);

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Aligned);
    Follower invertedFollower = new Follower(Constants.Arm.LT_PORT, MotorAlignmentValue.Opposed);
    rightTopMotor.setControl(follower);
    rightBottomMotor.setControl(follower);
    leftBottomMotor.setControl(invertedFollower);

    // Getting the General Configurator
    TalonFXConfigurator rightTopMotorConfig = rightTopMotor.getConfigurator();
    TalonFXConfigurator rightBottomMotorConfig = rightBottomMotor.getConfigurator();
    TalonFXConfigurator leftTopMotorConfig = leftTopMotor.getConfigurator();
    TalonFXConfigurator leftBottomMotorConfig = leftBottomMotor.getConfigurator();

    // Applying the Motor Output Configs to all four motors
    rightTopMotorConfig.apply(moc);
    rightBottomMotorConfig.apply(moc);
    leftTopMotorConfig.apply(moc);
    leftBottomMotorConfig.apply(moc);

    // Why do we apply Current Limit Configs to each motor, but then only do s0c on the
    // master?

    // TODO -> Answer the question here: 



    // Apply Current Limit to all motors
    rightTopMotorConfig.apply(clc);
    rightBottomMotorConfig.apply(clc);
    leftTopMotorConfig.apply(clc);
    leftBottomMotorConfig.apply(clc);

    // Assign master motor and apply Slot0Configs to master
    master = leftTopMotor;
    TalonFXConfigurator masterConfigurator = master.getConfigurator();
    masterConfigurator.apply(s0c);

    // Apply MotionMagicConfigs to master motor
    mmc = new MotionMagicConfigs();
    // TODO: Set MAX Velocity
    mmc.MotionMagicCruiseVelocity =
        Constants.Arm.MOTIONMAGIC_CRUISE_VELOCITY;

    // TODO: Set Max Acceleration / Decceleration
    mmc.MotionMagicAcceleration =
        Constants.Arm.MOTIONMAGIC_ACCEL;
   
    masterConfigurator.apply(mmc);

    // Initialize absolute encoder
    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    // ==== EXPLANATION: ====
    // getAbsolutePosition(): Absolute Encoder's current reading
    // ABSOLUTE_ENCODER_HORIZONTAL: What the Absolute Encoder reads at horizontal
    // ABSOLUTE_HORIZONTAL_OFFSET: Number of rotations to offset angle count from horizontal, to
    // avoid "pretzeling" or slamming into robot
    // What this next line does:
    // Uses the Absolute Encoder to set the position of the Master motor, so that when the Master
    // motor reads 0 rotations,
    // it represents the arm being at horizontal. After this next line runs, the Master motor's
    // encoder reading can be used
    // like expected, so you simply need to divide its reading by INTEGRATED_ARM_CONVERSION_FACTOR
    // to get the arm's angle in rotations.
    // ======================

    // In a new thread, wait until absolute encoder is connected.
    // Then, set Master's position to the absolute encoder's position, converted to motor rotations.
    // Finally, initialize the armHorizontalOffset variable, which is the positive offset the arm
    // reads
    // as its angle when it's horizontal, in rotations.
    new Thread(
            () -> {
              try {
                do {
                  Thread.sleep(250);
                } while (!revEncoder.isConnected());
                master.setPosition(
                    (getAbsolutePosition()) * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
                initialized = true;
                armHorizontalOffset =
                    Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET
                        / Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR;

              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            })
        .run();

    // POTENTIAL PROBLEM:
    /* The way we take the Absolute Encoder's offsets.
     */

    // targetDegrees = Constants.Arm.DEFAULT_ARM_ANGLE;
    targetDegrees = getCorrectedDegrees() + 10d;
    // targetDegrees = 70d;
    enableArm = false;
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void resetPosition() {
    if (revEncoder.isConnected()) {
      master.setPosition(
          (getAbsolutePosition()) * Constants.Arm.INTEGRATED_ABSOLUTE_CONVERSION_FACTOR);
    }
  }

  private void setPosition(double angleDegrees) {
    // TODO: Why is the min angle here 4 degrees, but the min angle in `setTargetDegrees` 1 degree?
    angleDegrees = MathUtil.clamp(angleDegrees, 3, 110);
    if (initialized && enableArm) {
      master.setControl(
          new MotionMagicVoltage(calculateIntegratedTargetRots(angleDegrees)));
    }
    // if(master.getVelocity().getValue() == 0){

    // }
  }

  public void setTargetDegrees(double angleDegrees) {
    targetDegrees = angleDegrees;
  }

  // private get

  public void rotateToRestPosition() {
    setTargetDegrees(Constants.Arm.DEFAULT_ARM_ANGLE);
  }

  private double getAbsolutePosition() {
    // uses the absolute encoder rotations to get the absolute position

    return (revEncoder.get() // revEncoder.getAbsolutePosition()
            - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL
            + Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET
            + 1d)
        % 1;
  }

  private double getMotorPosRotations() {
    if (!initialized) {
      System.out.println(
          "WARNING: Motor Position looked at, but initialization not complete yet. Returning 0");
      return 0;
    }
    return master.getPosition().getValueAsDouble();
  }

  // TODO: LOOK HERE BECAUSE THIS IS HOW MOTOR ROTATIONS ARE CONVERTED INTO DEGREES
  private double getArmPosRotations() {
    // uses motor position to return arm position in rotations by dividing by the conversion factor
    return getMotorPosRotations() / Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
  }

  public double getRawDegrees() {
    // uses the arm position in rotations to get the degrees
    return getArmPosRotations() * 360d;
  }

  private double calculateIntegratedTargetRots(double angleDegrees) {
    // gets the target rotations for the motor given a target angle
    double armRots = angleDegrees / 360d + armHorizontalOffset;
    return armRots * Constants.Arm.INTEGRATED_ARM_CONVERSION_FACTOR;
  }

  public double getCorrectedDegrees() {
    // gets the actual degrees of the arm using the raw degrees of motor and subtracting the known
    // offset
    return getRawDegrees() - armHorizontalOffset * 360d;
  }

  public boolean atTarget(double tolerance) {
    return Math.abs(targetDegrees - getCorrectedDegrees()) < tolerance;
  }

  public void setEnable(boolean toset) {
    this.enableArm = toset;
  }

  @Override
  public void periodic() {
    setPosition(targetDegrees);
    SmartDashboard.putString(
        "ARM Command",
        this.getCurrentCommand() == null ? "none" : this.getCurrentCommand().getName());
    SmartDashboard.putNumber("ARM Abs Enc Raw", revEncoder.get());
    SmartDashboard.putNumber("ARM Abs Enc Func", getAbsolutePosition());
    SmartDashboard.putNumber("ARM Integrated Rotations", getMotorPosRotations());
    SmartDashboard.putNumber(
        "ARM Integrated Current", master.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "ARM Integrated Error", master.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("ARM Arm Rotations", getArmPosRotations());
    SmartDashboard.putNumber("ARM Arm Degrees", getRawDegrees());

    SmartDashboard.putNumber("ARM Arm Degrees Corrected", getCorrectedDegrees());
    SmartDashboard.putNumber("ARM Target Degrees", targetDegrees);
    SmartDashboard.putString(
        "Current commannd ARM:",
        (getCurrentCommand() == null) ? "NULL" : getCurrentCommand().getName());
    SmartDashboard.putNumber(
        "ARM Target Integrated Rots", calculateIntegratedTargetRots(targetDegrees));
    SmartDashboard.putNumber("Master Velocity", master.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "ARM Abs enc deg",
        Units.rotationsToDegrees(getAbsolutePosition() - Constants.Arm.ABSOLUTE_HORIZONTAL_OFFSET)
            / Constants.Arm.ABSOLUTE_ARM_CONVERSION_FACTOR);
    SmartDashboard.putNumber("ARM updown adjustment", Constants.Arm.ARM_INTERMAP_OFFSET);



    DogLog.log("MD.6-lab/Current_Arm_Angle_Degrees", getCorrectedDegrees());
    SmartDashboard.putNumber("MD.6-lab/Target_Arm_Angle_Degrees", targetDegrees);
    periodicSignalLogger();
  }

  public void periodicSignalLogger() {
    DogLog.log("arm/abs_encoder_raw", getAbsolutePosition());
    DogLog.log("arm/integrated_current", master.getSupplyCurrent().getValueAsDouble());
    DogLog.log("arm/closed_loop_error", master.getClosedLoopError().getValueAsDouble());
    DogLog.log("arm/corrected_angle", getCorrectedDegrees());
    DogLog.log("arm/target_angle_deg", targetDegrees);
    DogLog.log("arm/current_velocity_rps", master.getVelocity().getValueAsDouble());
    // SignalLogger.writeDouble("ARM Abs Enc Func: ", getAbsolutePosition());
    // SignalLogger.writeDouble("ARM Integrated Current: ", master.getSupplyCurrent().getValue());
    // SignalLogger.writeDouble("ARM Integrated Error: ", master.getClosedLoopError().getValue());
    // SignalLogger.writeDouble("Arm Corrected Degrees", getCorrectedDegrees());
    // SignalLogger.writeDouble("Target Arm Degrees", targetDegrees);
    // SignalLogger.writeDouble("Master Velocity", master.getVelocity().getValue());
  }
}
