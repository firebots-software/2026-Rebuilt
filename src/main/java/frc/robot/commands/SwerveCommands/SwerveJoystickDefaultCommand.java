package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeVisionDetection;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickDefaultCommand extends SequentialCommandGroup {
  public SwerveJoystickDefaultCommand(
      DoubleSupplier frontBackFunction,
      DoubleSupplier leftRightFunction,
      DoubleSupplier rotationFunction,
      DoubleSupplier speedFunction,
      BooleanSupplier fieldRelativeFunction,
      BooleanSupplier doPointing,
      BooleanSupplier redSideIfPointing,
      CommandSwerveDrivetrain swerveSubsystem,
      IntakeVisionDetection intakeVision,
      BooleanSupplier doDriveAssist,
      BooleanSupplier intakeVisionLockout,
      BooleanSupplier intakeExtended) {
    addCommands(
        Commands.either(
            new SwerveJoystickCommand(
                frontBackFunction,
                leftRightFunction,
                rotationFunction,
                speedFunction, // slowmode when left shoulder is pressed, otherwise fast
                () -> true,
                doPointing, // joystick.a().getAsBoolean()
                redSideIfPointing,
                swerveSubsystem),
            new SwerveJoystickCommandWithCorrection(
                frontBackFunction,
                leftRightFunction,
                rotationFunction,
                speedFunction,
                () -> true,
                doPointing,
                redSideIfPointing,
                swerveSubsystem,
                intakeVision,
                doDriveAssist,
                intakeExtended),
            () -> !intakeVisionLockout.getAsBoolean()));
  }
}
