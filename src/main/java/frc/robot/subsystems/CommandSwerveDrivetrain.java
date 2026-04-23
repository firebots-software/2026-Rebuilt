package frc.robot.subsystems;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.MiscUtils;
import frc.robot.util.Targeting;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private Translation2d virtualTarget = null;
  private boolean virtualTargetComputedThisLoop = false;
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  private final PIDController m_pathXController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kPX,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kIX,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kDX);
  private final PIDController m_pathYController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kPY,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kIY,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kDY);
  private final PIDController m_pathThetaController =
      new PIDController(
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kPR,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kIR,
          Constants.Swerve.WHICH_SWERVE_ROBOT.CHOREO_PID_VALUES.kDR);


  private SwerveDriveState currentState;

  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  private final Field2d field = new Field2d();

  // private final StructPublisher<Pose2d> posePublisher =
  //     NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
  private PIDController headingPIDController =
      new PIDController(
          1.5, // 4 was good
          0, //
          0); // -13 was good
  // 15, 0, 0 w/o FF
  public DoubleSubscriber headingKPTunable =
      DogLog.tunable("Subsystems/Swerve/kPHeading", 15.0, headingPIDController::setP);
  public DoubleSubscriber headingKITunable =
      DogLog.tunable("Subsystems/Swerve/kIHeading", 0.0, headingPIDController::setI);
  public DoubleSubscriber headingKIDunable =
      DogLog.tunable("Subsystems/Swerve/kDHeading", 0.0, headingPIDController::setD);

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    currentState = getState();

    headingPIDController.setIZone(0.14);
    headingPIDController.setIntegratorRange(0.0, Math.PI / 4); // 0.3 before
    headingPIDController.enableContinuousInput(-Math.PI, Math.PI); // 0.3 before
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData(field);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    SmartDashboard.putData(field);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    SmartDashboard.putData(field);
  }

  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
    return new AutoFactory(
        () -> getCurrentState().Pose,
        this::resetPose,
        this::followPath,
        true,
        this,
        trajLogger); // getState().pose
  }

  public void followPath(SwerveSample sample) {
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI); // every run?

    Pose2d pose = getState().Pose;

    ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));

    DogLog.log("Subsystems/Swerve/SampleX", sample.x);
    DogLog.log("Subsystems/Swerve/SampleY", sample.y);
    DogLog.log("Subsystems/Swerve/SampleHeading", sample.heading);
    DogLog.log("Subsystems/Swerve/SampleVX", sample.vx);
    DogLog.log("Subsystems/Swerve/SampleVY", sample.vy);
    DogLog.log("Subsystems/Swerve/SampleAngularVelocity", sample.omega);
    DogLog.log("Subsystems/Swerve/SampleAX", sample.ax);
    DogLog.log("Subsystems/Swerve/SampleAY", sample.ay);
    DogLog.log("Subsystems/Swerve/SampleAngularAcceleration", sample.alpha);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Translation2d getVirtualTarget(BooleanSupplier redside, BooleanSupplier override) {
    if (override.getAsBoolean()) {
      Translation2d target =
          (redside.getAsBoolean())
              ? (Constants.Landmarks.RED_HUB.getTranslation())
              : (Constants.Landmarks.BLUE_HUB.getTranslation());
      DogLog.log("VirtualTarget", new Pose2d(target, new Rotation2d()));
      return target;
    }
    if (virtualTarget == null || !virtualTargetComputedThisLoop) {
      virtualTarget = Targeting.computeVirtualTarget(Targeting.getHub(redside), this);
      virtualTargetComputedThisLoop = true;
    }
    DogLog.log("VirtualTarget", new Pose2d(virtualTarget, new Rotation2d()));
    return virtualTarget;
  }

  public Translation2d getPassingTarget(BooleanSupplier redside) {
    return redside.getAsBoolean()
        ? this.getPose()
            .nearest(
                Arrays.asList(Constants.Landmarks.RED_PASSING_L, Constants.Landmarks.RED_PASSING_R))
            .getTranslation()
        : this.getPose()
            .nearest(
                Arrays.asList(
                    Constants.Landmarks.BLUE_PASSING_L, Constants.Landmarks.BLUE_PASSING_R))
            .getTranslation();
  }

  public SwerveDriveState getCurrentState() {
    return currentState;
  }

  public ChassisSpeeds getRobotSpeeds() {
    return currentState.Speeds;
  }

  public void applyFieldSpeeds(
      ChassisSpeeds speeds, WheelForceCalculator.Feedforwards feedforwards) {
    setControl(
        m_pathApplyFieldSpeeds
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.x_newtons)
            .withWheelForceFeedforwardsY(feedforwards.y_newtons));
  }

  public void applyOneFieldSpeeds(ChassisSpeeds speeds) {
    setControl(
        m_pathApplyFieldSpeeds.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity));
  }

  public double getSpeedMagnitude() {
    double xSpeed = getFieldSpeeds().vxMetersPerSecond;
    double ySpeed = getFieldSpeeds().vyMetersPerSecond;
    return Math.sqrt((xSpeed * xSpeed) + (ySpeed * ySpeed));
  }

  public Pose2d getPose() {
    return currentState.Pose;
  }

  public double distanceToPose(Pose2d target) {
    return getPose().getTranslation().getDistance(target.getTranslation());
  }

  public Rotation2d travelAngleTo(Pose2d targetPose) {
    double deltaX = targetPose.getX() - getCurrentState().Pose.getX();
    double deltaY = targetPose.getY() - getCurrentState().Pose.getY();
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void periodic() {
    virtualTargetComputedThisLoop = false;
    DogLog.log("SpeedMagnitude", getSpeedMagnitude());
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    currentState = getState();
    DogLog.log("Subsystems/Swerve/AccumulatedError", headingPIDController.getAccumulatedError());

    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }

    if (this.getCurrentCommand() != null) {
      DogLog.log("Subsystems/Swerve/CurrentCommand", this.getCurrentCommand().toString());
    }
    DogLog.log("Subsystems/Swerve/Pose", getCurrentState().Pose);

    DogLog.log("Subsystems/Swerve/CurrPoseX", getCurrentState().Pose.getX());
    DogLog.log("Subsystems/Swerve/CurrPoseY", getCurrentState().Pose.getY());
    DogLog.log("Subsystems/Swerve/CurrPoseRotRads", getCurrentState().Pose.getRotation());
    DogLog.log(
        "Subsystems/Swerve/CurrPoseRotDegs", getCurrentState().Pose.getRotation().getDegrees());

    DogLog.log(
        "Subsystems/Swerve/DistanceToHub",
        MiscUtils.getDistanceToHub(RobotContainer::isRedAlliance, this));
    DogLog.log("Subsystems/Swerve/TurningSpeedActual", getFieldSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPose, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPose, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        currentState.Speeds, currentState.Pose.getRotation());
  }

  public double calculateRequiredRotationalRate(Rotation2d targetRotation) {
    double omega =
        // headingProfiledPIDController.getSetpoint().velocity+
        headingPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetRotation.getRadians());
    double max = Constants.Swerve.MAX_HEADING_TRACKING_ROT_RATE_RADS_PER_SECOND;
    boolean clamp = Math.abs(omega) > max;
    if (clamp) {
      omega = MathUtil.clamp(omega, -max, max);
    }
    DogLog.log("Subsystems/Swerve/RotationController/clamped", clamp);
    DogLog.log("Subsystems/Swerve/TargetRotationsDegrees", targetRotation.getDegrees());
    return omega;
  }

  public double calculateRequiredRotationalRateWithFF(Translation2d targetPoint) {
    Translation2d robotPos = getCurrentState().Pose.getTranslation();
    double dx = targetPoint.getX() - robotPos.getX();
    double dy = targetPoint.getY() - robotPos.getY();
    double r2 = dx * dx + dy * dy;

    double omegaFF = 0.0;
    if (r2 > Constants.Swerve.FF_RADIUS_M2) {
      ChassisSpeeds fieldSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              currentState.Speeds, currentState.Pose.getRotation());
      double vx = fieldSpeeds.vxMetersPerSecond;
      double vy = fieldSpeeds.vyMetersPerSecond;

      omegaFF = (dy * vx - dx * vy) / r2;
    }

    Rotation2d targetRotation = new Rotation2d(Math.atan2(dy, dx) + Math.PI);
    double omegaPID =
        headingPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetRotation.getRadians());

    // double sign = 1;
    // if (omegaPID < 0) {
    //   sign = -1;
    // }

    double angleDifference =
        Math.atan2(
            Math.sin(targetRotation.getRadians() - currentState.Pose.getRotation().getRadians()),
            Math.cos(targetRotation.getRadians() - currentState.Pose.getRotation().getRadians()));

    // if ((Math.abs(angleDifference) < 0.87) && Math.abs(omegaPID) >= 1.295) {
    //   omegaPID = Math.abs(angleDifference) * 2.2 * sign;
    // }

    if (Math.abs(angleDifference) < 0.52) {
      omegaFF = 0;
    }

    double omega = (omegaFF) + omegaPID;

    DogLog.log("Subsystems/Swerve/RotationController/omegaFF", omegaFF);
    DogLog.log("Subsystems/Swerve/RotationController/omegaPID", omegaPID);
    DogLog.log("Subsystems/Swerve/TargetRotationsDegrees", targetRotation.getDegrees());
    DogLog.log("AngleDifference", angleDifference);
    return omega;
  }

  public Command brakeSwerve() {
    return run(() -> setControl(m_brake));
  }
}
