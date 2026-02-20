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
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

  private TalonFXSimState sitUpMotorSimState;
  private TalonFXSimState muscleUpMotorSimState;
  private TalonFXSimState pullUpMotorSimState; // only simulating one of the pull up motors

  private CANcoderSimState sitUpEncoderSimState;

  // SingleJointedArmSim for the two rotating mechanisms
  private SingleJointedArmSim sitUpMechanismSim;
  private SingleJointedArmSim muscleUpMechanismSim;

  // ElevatorSim for the linear belt-driven elevator (pull up)
  private ElevatorSim pullUpMechanismSim;

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
            .withKV(Constants.Climber.KV)
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

    if (RobotBase.isSimulation()) setupSimulation();
  }

  // ---------------------------------------------------------------------------
  // Simulation setup — modeled after IntakeSubsystem.setupSimulation()
  // ---------------------------------------------------------------------------
  private void setupSimulation() {
    // Grab sim states for the three driven motors and both CANcoders
    sitUpMotorSimState = sitUpMotor.getSimState();
    muscleUpMotorSimState = muscleUpMotor.getSimState();
    pullUpMotorSimState = pullUpMotorR.getSimState(); // follower (L) mirrors leader automatically

    sitUpEncoderSimState = sitUpEncoder.getSimState();
    muscleUpEncoderSimState = muscleUpEncoder.getSimState();

    // Match the real motor sign conventions set in the configurators above
    sitUpMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    muscleUpMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    pullUpMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;

    // Motor types from the requirements doc:
    //   Sit up   → Kraken X44   (1× motor in subsystem)
    //   Muscle up → Kraken X60  (1× motor in subsystem)
    //   Pull up  → Kraken X60   (2× motors, but we only drive the leader in sim)
    sitUpMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    muscleUpMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    pullUpMotorSimState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    // Gearbox models
    var sitUpGearbox = DCMotor.getKrakenX44Foc(1);
    var muscleUpGearbox = DCMotor.getKrakenX60Foc(1);
    // Pull-up uses 2 motors mechanically coupled; model as 2-motor gearbox for accuracy
    var pullUpGearbox = DCMotor.getKrakenX60Foc(2);

    // --- Sit Up: rotational arm pivot, 102.4:1 end-to-end reduction ---
    sitUpMechanismSim =
        new SingleJointedArmSim(
            sitUpGearbox,
            Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT,
            Constants.Climber.SitUp.SIM_MOI_KG_M2,
            Constants.Climber.SitUp.SIM_ARM_LENGTH_METERS,
            Units.degreesToRadians(Constants.Climber.SitUp.SIM_MIN_ANGLE_DEG),
            Units.degreesToRadians(Constants.Climber.SitUp.SIM_MAX_ANGLE_DEG),
            true, // simulate gravity
            Units.degreesToRadians(Constants.Climber.SitUp.SIM_START_ANGLE_DEG));

    // --- Muscle Up: rotational pivot, 124.583:1 end-to-end reduction ---
    muscleUpMechanismSim =
        new SingleJointedArmSim(
            muscleUpGearbox,
            Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT,
            Constants.Climber.MuscleUp.SIM_MOI_KG_M2,
            Constants.Climber.MuscleUp.SIM_ARM_LENGTH_METERS,
            Units.degreesToRadians(Constants.Climber.MuscleUp.SIM_MIN_ANGLE_DEG),
            Units.degreesToRadians(Constants.Climber.MuscleUp.SIM_MAX_ANGLE_DEG),
            true, // simulate gravity
            Units.degreesToRadians(Constants.Climber.MuscleUp.SIM_START_ANGLE_DEG));

    // --- Pull Up: linear belt-driven elevator, 18.225:1 reduction ---
    // Modeled as a DCMotorSim since the output is linear motion, not rotation.
    // MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL converts mechanism meters → motor rotations.
    pullUpMechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                pullUpGearbox,
                Constants.Climber.PullUp.SIM_MOI_KG_M2,
                Constants.Climber.PullUp.MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL),
            pullUpGearbox);
  }

  // ---------------------------------------------------------------------------
  // Existing public methods — unchanged from original
  // ---------------------------------------------------------------------------

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

  // Commands
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

  // ---------------------------------------------------------------------------
  // Periodic
  // ---------------------------------------------------------------------------

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

  // ---------------------------------------------------------------------------
  // Simulation periodic — modeled after IntakeSubsystem.simulationPeriodic()
  // ---------------------------------------------------------------------------

  @Override
  public void simulationPeriodic() {
    // Guard: if setupSimulation() was never called (non-sim robot), bail out.
    if (sitUpMotorSimState == null
        || muscleUpMotorSimState == null
        || pullUpMotorSimState == null
        || sitUpMechanismSim == null
        || muscleUpMechanismSim == null
        || pullUpMechanismSim == null) {
      return;
    }

    // 1) Push current battery voltage into all motor sim states
    double batteryV = RobotController.getBatteryVoltage();
    sitUpMotorSimState.setSupplyVoltage(batteryV);
    muscleUpMotorSimState.setSupplyVoltage(batteryV);
    pullUpMotorSimState.setSupplyVoltage(batteryV);

    // 2) Read applied voltages from each motor controller and step the mechanism plants
    double sitUpAppliedVolts =
        sitUpMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    double muscleUpAppliedVolts =
        muscleUpMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);
    // Pull-up: only leader (R) is actively controlled; follower (L) mirrors it.
    double pullUpAppliedVolts =
        pullUpMotorSimState.getMotorVoltageMeasure().in(edu.wpi.first.units.Units.Volts);

    sitUpMechanismSim.setInputVoltage(sitUpAppliedVolts);
    muscleUpMechanismSim.setInputVoltage(muscleUpAppliedVolts);
    pullUpMechanismSim.setInputVoltage(pullUpAppliedVolts);

    sitUpMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    muscleUpMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);
    pullUpMechanismSim.update(Constants.Simulation.SIM_LOOP_PERIOD_SECONDS);

    // 3) Feed mechanism-side state back into the Talon rotor sensors

    // --- Sit Up (SingleJointedArmSim outputs radians) ---
    double sitUpMechAngleRad = sitUpMechanismSim.getAngleRads();
    double sitUpMechVelRps = sitUpMechanismSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    double sitUpMechPosRot = sitUpMechAngleRad / (2.0 * Math.PI);

    double sitUpRotorPosRot = sitUpMechPosRot * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;
    double sitUpRotorVelRps = sitUpMechVelRps * Constants.Climber.SitUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;

    sitUpMotorSimState.setRawRotorPosition(sitUpRotorPosRot);
    sitUpMotorSimState.setRotorVelocity(sitUpRotorVelRps);

    // --- Muscle Up (SingleJointedArmSim outputs radians) ---
    double muscleUpMechAngleRad = muscleUpMechanismSim.getAngleRads();
    double muscleUpMechVelRps = muscleUpMechanismSim.getVelocityRadPerSec() / (2.0 * Math.PI);
    double muscleUpMechPosRot = muscleUpMechAngleRad / (2.0 * Math.PI);

    double muscleUpRotorPosRot = muscleUpMechPosRot * Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;
    double muscleUpRotorVelRps = muscleUpMechVelRps * Constants.Climber.MuscleUp.MOTOR_ROTS_PER_DEGREES_OF_ARM_ROT;

    muscleUpMotorSimState.setRawRotorPosition(muscleUpRotorPosRot);
    muscleUpMotorSimState.setRotorVelocity(muscleUpRotorVelRps);

    // --- Pull Up (DCMotorSim; mechanism side = belt meters, rotor side = motor rotations) ---
    double pullUpMechPosRot = pullUpMechanismSim.getAngularPositionRotations();
    double pullUpMechVelRps = pullUpMechanismSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    double pullUpRotorPosRot = pullUpMechPosRot * Constants.Climber.PullUp.MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL;
    double pullUpRotorVelRps = pullUpMechVelRps * Constants.Climber.PullUp.MOTOR_ROTS_PER_METERS_OF_BELT_TRAVERSAL;

    pullUpMotorSimState.setRawRotorPosition(pullUpRotorPosRot);
    pullUpMotorSimState.setRotorVelocity(pullUpRotorVelRps);

    // 4) Keep both CANcoders in sync with their respective mechanism positions/velocities
    //    CANcoder sees arm-side rotations, not rotor-side rotations.
    sitUpEncoderSimState.setRawPosition(
        sitUpMechPosRot * Constants.Climber.SitUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS);
    sitUpEncoderSimState.setVelocity(
        sitUpMechVelRps * Constants.Climber.SitUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS);

    muscleUpEncoderSimState.setRawPosition(
        muscleUpMechPosRot * Constants.Climber.MuscleUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS);
    muscleUpEncoderSimState.setVelocity(
        muscleUpMechVelRps * Constants.Climber.MuscleUp.ENCODER_ROTATIONS_TO_ARM_ROTATIONS);

    // 5) Battery sag model — sum supply current from all three motors
    //    Pull-up uses 2 physical motors; double the leader's supply current to account for both.
    double totalSupplyCurrentAmps =
        sitUpMotorSimState.getSupplyCurrent()
            + muscleUpMotorSimState.getSupplyCurrent()
            + (pullUpMotorSimState.getSupplyCurrent() * 2);

    double loadedBatteryV = BatterySim.calculateDefaultBatteryLoadedVoltage(totalSupplyCurrentAmps);
    RoboRioSim.setVInVoltage(loadedBatteryV);
  }
}