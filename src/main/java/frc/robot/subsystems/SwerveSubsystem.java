package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.Supplier;

// import choreo.trajectory.SwerveSample;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem {
  private static SwerveSubsystem instance;

  private ProfiledPIDController qProfiledPIDController, headingProfiledPIDController;

  private SwerveDriveState currentState;

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double OdometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        drivetrainConstants,
        OdometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    currentState = getState(); // getCurrentState
    // 1.7, 0.345, 0.0015
    qProfiledPIDController =
        new ProfiledPIDController(
            3.4, // 3.4 not bad // [3.4 good for 0.2-1.2, 0.425 I]
            0.45, // 345
            0.0005, // 0.0015
            new TrapezoidProfile.Constraints(
                Constants.Swerve.PHYSICAL_MAX_SPEED_METERS_PER_SECOND - 0.5,
                8)); // 8.25 // 5 accel and 0.75 p was good

    headingProfiledPIDController =
        new ProfiledPIDController(
            3.7, // 4 was good
            0.4, //
            0,
            new TrapezoidProfile.Constraints(
                Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_RATE - 1.5, // -1 was good
                Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND
                    - 16)); // -13 was good
    // headingProfiledPIDController =
    //     new ProfiledPIDController(
    //         1, // 4 was good
    //         0.2, //
    //         0,
    //         new TrapezoidProfile.Constraints(
    //             Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_RATE - 1.5, // -1 was good
    //             Constants.Swerve.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND
    //                 - 16)); // -13 was good

    qProfiledPIDController.setIZone(0.35);
    headingProfiledPIDController.setIZone(0.14);

    qProfiledPIDController.setIntegratorRange(0, 0.2);
    headingProfiledPIDController.setIntegratorRange(0.0, Math.PI / 4); // 0.3 before

    headingProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
    // configureAutoBuilder();
  }

  // Values relevant for the simulation
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // starts the simulator thread
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /** Swerve request to apply during field-centric path following */
  private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds();

  // private void configureAutoBuilder() {
  //   try {
  //     var config = RobotConfig.fromGUISettings();
  //     AutoBuilder.configure(
  //         () -> currentState.Pose, // Supplier of current robot pose
  //         this::resetPose, // Consumer for seeding pose against auto
  //         () -> currentState.Speeds, // Supplier of current robot speeds
  //         // Consumer of ChassisSpeeds and feedforwards to drive the robot
  //         (speeds, feedforwards) ->
  //             setControl(
  //                 m_pathApplyRobotSpeeds
  //                     .withSpeeds(speeds)
  //                     .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
  //                     .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
  //         new PPHolonomicDriveController(
  //             // PID constants for translation
  //             new PIDConstants(10, 0, 0),
  //             // PID constants for rotation
  //             new PIDConstants(7, 0, 0)),
  //         config,
  //         // Assume the path needs to be flipped for Red vs Blue, this is normally the case
  //         () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
  //         this // Subsystem for requirements
  //         );
  //   } catch (Exception ex) {
  //     DriverStation.reportError(
  //         "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
  //   }
  // }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance =
          new SwerveSubsystem(
              Constants.Swerve.DrivetrainConstants,
              250,
              Constants.Kalman.odometryMatrix,
              Constants.Kalman.visionMatrix,
              Constants.Swerve.FrontLeft,
              Constants.Swerve.FrontRight,
              Constants.Swerve.BackLeft,
              Constants.Swerve.BackRight);
    }
    return instance;
  }

  public double getDirectionalChassisSpeeds(Rotation2d qDirection) {
    return (qDirection.getCos() * getFieldSpeeds().vxMetersPerSecond)
        + (qDirection.getSin() * getFieldSpeeds().vyMetersPerSecond);
  }

  public void resetRotationPID() {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
  }

  // Resets PID controllers
  public void resetProfiledPIDs() {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
  }

  public void resetProfiledPIDs(Rotation2d qDirection) {
    headingProfiledPIDController.reset(
        currentState.Pose.getRotation().getRadians(), getFieldSpeeds().omegaRadiansPerSecond);
    qProfiledPIDController.reset(0, getDirectionalChassisSpeeds(qDirection));
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Getter for Robot's SwerveDriveState
   *
   * @return Robot's current SwerveDriveState
   */
  public SwerveDriveState getCurrentState() {
    return currentState;
  }

  /**
   * @return Robot's current Robot Chassis Speeds
   */
  public ChassisSpeeds getRobotSpeeds() {
    return currentState.Speeds;
  }

  /**
   * @return Robot's current Field-Centric Chassis Speeds
   */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), currentState.Pose.getRotation());
  }

  public void setRobotSpeeds(ChassisSpeeds speeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
  }

  public void setFieldSpeeds(ChassisSpeeds speeds) {
    setControl(
        m_pathApplyFieldSpeeds.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity));
  }

  public Pose2d getPose() {
    return currentState.Pose;
  }

  public Rotation2d travelAngleTo(Pose2d targetPose) {
    double deltaX = targetPose.getX() - getCurrentState().Pose.getX();
    double deltaY = targetPose.getY() - getCurrentState().Pose.getY();
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  public double calculateRequiredRotationalRate(Rotation2d targetRotation) {
    double omega =
        // headingProfiledPIDController.getSetpoint().velocity+
        headingProfiledPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetRotation.getRadians());
    return omega;
  }

  public ChassisSpeeds calculateRequiredEdwardChassisSpeeds(
      Pose2d targetPose, double completePathDistance) {
    double distanceToTarget =
        getCurrentState().Pose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (distanceToTarget - Constants.HardenConstants.ffMinRadius)
                / (Constants.HardenConstants.ffMaxRadius - Constants.HardenConstants.ffMinRadius),
            0.0,
            1.0);

    double qSpeed =
        (qProfiledPIDController.getSetpoint().velocity * ffScaler)
            + qProfiledPIDController.calculate(
                completePathDistance - distanceToTarget, completePathDistance);
    double omega =
        // headingProfiledPIDController.getSetpoint().velocity+
        headingProfiledPIDController.calculate(
            currentState.Pose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Rotation2d travelAngle = travelAngleTo(targetPose);

    DogLog.log(
        "EdwardCalculation/qProfiledPID/CurrentPositionMeasurement",
        completePathDistance - distanceToTarget);
    DogLog.log(
        "EdwardCalculation/qProfiledPID/PositionSetpoint",
        qProfiledPIDController.getSetpoint().position);
    DogLog.log(
        "EdwardCalculation/qProfiledPID/CurrentVelocityMeasurement",
        getDirectionalChassisSpeeds(travelAngle));
    DogLog.log(
        "EdwardCalculation/qProfiledPID/VelocitySetpoint",
        qProfiledPIDController.getSetpoint().velocity);
    DogLog.log(
        "EdwardCalculation/qProfiledPID/PositionError", qProfiledPIDController.getPositionError());
    DogLog.log(
        "EdwardCalculation/qProfiledPID/VelocityError", qProfiledPIDController.getVelocityError());

    DogLog.log(
        "EdwardCalculation/headingProfiledPID/CurrentPositionMeasurement",
        currentState.Pose.getRotation().getRadians());
    DogLog.log(
        "EdwardCalculation/headingProfiledPID/PositionSetpoint",
        headingProfiledPIDController.getSetpoint().position);
    DogLog.log(
        "EdwardCalculation/headingProfiledPID/CurrentVelocityMeasurement",
        currentState.Speeds.omegaRadiansPerSecond);
    DogLog.log(
        "EdwardCalculation/headingProfiledPID/VelocitySetpoint",
        headingProfiledPIDController.getSetpoint().velocity);
    DogLog.log(
        "EdwardCalculation/headingProfiledPID/PositionError",
        headingProfiledPIDController.getPositionError());
    DogLog.log(
        "EdwardCalculation/headingProfiledPID/VelocityError",
        headingProfiledPIDController.getVelocityError());

    return new ChassisSpeeds(
        qSpeed * Math.cos(travelAngle.getRadians())
            + (0 * 0.075 * Math.cos(getCurrentState().Pose.getRotation().getRadians())),
        qSpeed * Math.sin(travelAngle.getRadians())
            + (0 * 0.075 * Math.sin(getCurrentState().Pose.getRotation().getRadians())),
        omega);
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

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());
    DogLog.log(
        "subsystems/swerve/module0/drive/speedrps",
        this.getModule(1).getDriveMotor().getVelocity().getValueAsDouble());

    currentState = getState();

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

    DogLog.log("subsystems/swerve/fl_speed", currentState.ModuleStates[0].speedMetersPerSecond);
    DogLog.log("subsystems/swerve/fl_angle", currentState.ModuleStates[0].angle.getDegrees());
    DogLog.log("subsystems/swerve/fr_speed", currentState.ModuleStates[1].speedMetersPerSecond);
    DogLog.log("subsystems/swerve/fr_angle", currentState.ModuleStates[1].angle.getDegrees());
    DogLog.log("subsystems/swerve/bl_speed", currentState.ModuleStates[2].speedMetersPerSecond);
    DogLog.log("subsystems/swerve/bl_angle", currentState.ModuleStates[2].angle.getDegrees());
    DogLog.log("subsystems/swerve/br_speed", currentState.ModuleStates[3].speedMetersPerSecond);
    DogLog.log("subsystems/swerve/br_angle", currentState.ModuleStates[3].angle.getDegrees());
    DogLog.log(
        "subsystems/swerve/command",
        this.getCurrentCommand() == null ? "NOTHING" : this.getCurrentCommand().getName());
  }
}
