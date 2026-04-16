package frc.robot.commandGroups.ShootCommandGroups;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToAndShoot extends ParallelCommandGroup {

  public DriveToAndShoot(
      Supplier<Pose2d> targetPoseSupplier,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      HopperSubsystem hopperSubsystem,
      CommandSwerveDrivetrain drivetrain,
      BooleanSupplier redside) {

    DoubleSupplier dist =
        () ->
            drivetrain
                .getPose()
                .getTranslation()
                .getDistance(drivetrain.getVirtualTarget(redside, () -> false));

    addCommands(
        new DriveToPoseAimed(drivetrain, targetPoseSupplier, redside),
        shooterSubsystem.shootAtSpeedHoodCommand(
            () -> shooterSubsystem.grabTargetShootingSpeed(dist.getAsDouble()),
            () -> shooterSubsystem.grabTargetHoodAngle(dist.getAsDouble())),
        Commands.waitUntil(shooterSubsystem::isAtSpeed)
            .andThen(
                Commands.parallel(
                    hopperSubsystem.runHopperUntilInterruptedCommand(
                        () -> Constants.Hopper.TARGET_SURFACE_SPEED_MPS,
                        () ->
                            Targeting.pointingAtTarget(
                                drivetrain
                                        .travelAngleTo(
                                            new Pose2d(
                                                drivetrain.getVirtualTarget(redside, () -> false),
                                                new Rotation2d()))
                                        .getRadians()
                                    + Math.PI,
                                drivetrain)),
                    intakeSubsystem.powerRetractRollersCommand())));
  }

  private static final class DriveToPoseAimed extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final BooleanSupplier redside;

    private LinearPath path = null;
    private LinearPath.State pathState = null;
    private Pose2d targetPose = null;

    private final PIDController xController =
        new PIDController(
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kPX,
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kIX,
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kDX);
    private final PIDController yController =
        new PIDController(
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kPY,
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kIY,
            Constants.Swerve.WHICH_SWERVE_ROBOT.SWERVE_DRIVE_TO_POSE_PID_VALUES.kDY);

    private double startTime;

    DriveToPoseAimed(
        CommandSwerveDrivetrain swerve,
        Supplier<Pose2d> targetPoseSupplier,
        BooleanSupplier redside) {
      this.swerve = swerve;
      this.targetPoseSupplier = targetPoseSupplier;
      this.redside = redside;

      path =
          new LinearPath(
              new TrapezoidProfile.Constraints(1.5, 2.0),
              new TrapezoidProfile.Constraints(
                  Constants.Swerve.WHICH_SWERVE_ROBOT
                      .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                      .maxVelocityAngular,
                  Constants.Swerve.WHICH_SWERVE_ROBOT
                      .SWERVE_DRIVE_TO_POSE_PROFILE_VALUES
                      .maxAccelerationAngular));

      addRequirements(swerve);
    }

    @Override
    public void initialize() {
      startTime = Utils.getCurrentTimeSeconds();
      targetPose = targetPoseSupplier.get();

      pathState =
          new LinearPath.State(swerve.getCurrentState().Pose, swerve.getCurrentState().Speeds);

      DogLog.log("Subsystems/Swerve/DriveToAndShoot/InitTargetPoseX", targetPose.getX());
      DogLog.log("Subsystems/Swerve/DriveToAndShoot/InitTargetPoseY", targetPose.getY());
    }

    @Override
    public void execute() {
      if (pathState == null) return;

      double currTime = Utils.getCurrentTimeSeconds() - startTime;

      pathState = path.calculate(currTime, pathState, targetPose);

      double vx =
          pathState.speeds.vxMetersPerSecond
              + xController.calculate(swerve.getCurrentState().Pose.getX(), pathState.pose.getX());
      double vy =
          pathState.speeds.vyMetersPerSecond
              + yController.calculate(swerve.getCurrentState().Pose.getY(), pathState.pose.getY());

      double omega =
          swerve.calculateRequiredRotationalRateWithFF(
              swerve.getVirtualTarget(redside, () -> false));

      swerve.applyOneFieldSpeeds(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return Math.abs(swerve.getCurrentState().Pose.getX() - targetPose.getX())
              <= Constants.Swerve.TARGET_POS_ERROR
          && Math.abs(swerve.getCurrentState().Pose.getY() - targetPose.getY())
              <= Constants.Swerve.TARGET_POS_ERROR;
    }
  }
}
