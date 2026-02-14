package frc.robot.commands.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurnTowardsTarget extends Command {
  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

  protected final CommandSwerveDrivetrain swerveDrivetrain;

  public TurnTowardsTarget(CommandSwerveDrivetrain drivetrain) {
    swerveDrivetrain = drivetrain;
  }

  public void execute() {
    double turn =
        swerveDrivetrain.calculateRequiredRotationalRate(new Rotation2d(Shoot.targetAngle));
    SwerveRequest drive =
        fieldCentricDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(turn);
    this.swerveDrivetrain.setControl(drive);
  }

  public boolean isFinished() {
    return Shoot.pointingAtTarget(swerveDrivetrain);
  }
}
