// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final HopperSubsystem hopperSubsystem =
      Constants.hopperOnRobot ? new HopperSubsystem() : null;
  public final IntakeSubsystem intakeSubsystem =
      Constants.intakeOnRobot ? new IntakeSubsystem() : null;
  public final ShooterSubsystem lebron = Constants.shooterOnRobot ? new ShooterSubsystem() : null;

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // INTAKE COMMANDS
    // right bumper -> run intake
    if (Constants.intakeOnRobot) {
      joystick.rightBumper().whileTrue(intakeSubsystem.runIntake());

      // left trigger + x -> arm to initial pos (0)
      joystick
          .leftTrigger()
          .and(joystick.x())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_INITIAL));

      // left trigger + a -> arm to extended pos (15)
      joystick
          .leftTrigger()
          .and(joystick.a())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_EXTENDED));

      // left trigger + b -> arm to idle pos (45)
      joystick
          .leftTrigger()
          .and(joystick.b())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_IDLE));

      // left trigger + y -> arm to retracted pos (90)
      joystick
          .leftTrigger()
          .and(joystick.y())
          .onTrue(intakeSubsystem.armToDegrees(Constants.Intake.Arm.ARM_POS_RETRACTED));
    }

    if (Constants.shooterOnRobot) {
      joystick.rightTrigger().whileTrue(lebron.ShootAtSpeed());
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
