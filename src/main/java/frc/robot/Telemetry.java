package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

public class Telemetry {
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
    SignalLogger.start();
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable driveStateTable = inst.getTable("DriveState");
  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisVelocities> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisVelocities.struct).publish();
  private final StructArrayPublisher<SwerveModuleVelocity> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModuleVelocity> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.WHITE))),
      };

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Velocity);
    driveModuleStates.set(state.ModuleVelocities);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    /* Also write to log file */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; ++i) {
      m_moduleStatesArray[i * 2 + 0] = state.ModuleVelocities[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleVelocities[i].velocity;
      m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].velocity;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.set("Field2d");
    fieldPub.set(m_poseArray);

    /* Telemeterize the module states to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleVelocities[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleVelocities[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleVelocities[i].velocity / (2 * MaxSpeed));

      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
  }
}
