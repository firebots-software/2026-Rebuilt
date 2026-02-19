package frc.robot;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Swerve.Auto.ClimbPos;
import frc.robot.Constants.Swerve.Auto.Intake;
import frc.robot.Constants.Swerve.Auto.Maneuver;
import frc.robot.Constants.Swerve.Auto.ShootPos;
import frc.robot.commandGroups.BumpDTP;
import frc.robot.commandGroups.ClimbCommands.L1Climb;
import frc.robot.commandGroups.ExtendIntake;
import frc.robot.commandGroups.RetractIntake;
import frc.robot.commandGroups.ShootBasic;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Targeting;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem lebronShooterSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final CommandSwerveDrivetrain swerveSubsystem;
  private final ClimberSubsystem climberSubsystem;

  public AutoRoutines(
      IntakeSubsystem intake,
      ShooterSubsystem lebron,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerve,
      ClimberSubsystem climber) {
    this.intakeSubsystem = intake;
    this.lebronShooterSubsystem = lebron;
    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    this.climberSubsystem = climber;

    autoFactory = swerveSubsystem.createAutoFactory();

    autoChooser = new AutoChooser();
    addCommandstoAutoChooser();
  }

  private AutoTrajectory maneuver(AutoRoutine routine, Maneuver type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    return traj;
  }

  private AutoTrajectory intake(AutoRoutine routine, Intake type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
      traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
    }

    return traj;
  }

  private AutoTrajectory shoot(AutoRoutine routine, ShootPos type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    return traj;
  }

  private AutoTrajectory climb(AutoRoutine routine, ClimbPos type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    return traj;
  }

  public Command RedPedri(
      Maneuver maneuverType, Intake intakeType, ShootPos shootType, ClimbPos climbType) {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory maneuver = maneuver(routine, maneuverType);
    AutoTrajectory intake = intake(routine, intakeType);
    AutoTrajectory shootPos = shoot(routine, shootType);
    AutoTrajectory climbPos = climb(routine, climbType);

    // add proper dtp
    routine
        .active()
        .onTrue(
            (maneuver != null ? maneuver.resetOdometry() : Commands.none())
                .andThen(getPathCommandSafely(maneuver))
                .andThen(() -> new BumpDTP(swerveSubsystem, () -> true))
                .andThen(getPathCommandSafely(intake))
                .andThen(() -> new BumpDTP(swerveSubsystem, () -> false))
                .andThen(getPathCommandSafely(shootPos))
                .andThen(
                    new ShootBasic(
                        () ->
                            Units.metersToFeet(
                                Targeting.shootingSpeed(
                                    Constants.Landmarks.RED_HUB,
                                    swerveSubsystem,
                                    Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
                        () ->
                            (Targeting.pointingAtTarget(
                                    Constants.Landmarks.RED_HUB, swerveSubsystem)
                                && lebronShooterSubsystem.isAtSpeed()),
                        lebronShooterSubsystem,
                        intakeSubsystem,
                        hopperSubsystem))
                .andThen(getPathCommandSafely(climbPos))
                .andThen(new L1Climb(climberSubsystem, swerveSubsystem)));

    return routine.cmd();
  }

  public Command Fermin(
      Maneuver maneuverType, Intake intakeType, ShootPos shootType, ClimbPos climbType) {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory maneuver = maneuver(routine, maneuverType);
    AutoTrajectory intake = intake(routine, intakeType);
    AutoTrajectory shootPos = shoot(routine, shootType);
    AutoTrajectory climbPos = climb(routine, climbType);

    routine
        .active()
        .onTrue(
            (maneuver != null ? maneuver.resetOdometry() : Commands.none())
                .andThen(getPathCommandSafely(maneuver))
                .andThen(() -> new BumpDTP(swerveSubsystem, () -> true))
                .andThen(getPathCommandSafely(intake))
                .andThen(() -> new BumpDTP(swerveSubsystem, () -> false))
                .andThen(getPathCommandSafely(shootPos))
                .andThen(
                    new ShootBasic(
                        () ->
                            Units.metersToFeet(
                                Targeting.shootingSpeed(
                                    Constants.Landmarks.RED_HUB,
                                    swerveSubsystem,
                                    Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
                        () ->
                            (Targeting.pointingAtTarget(
                                    Constants.Landmarks.RED_HUB, swerveSubsystem)
                                && lebronShooterSubsystem.isAtSpeed()),
                        lebronShooterSubsystem,
                        intakeSubsystem,
                        hopperSubsystem))
                .andThen(getPathCommandSafely(climbPos))
                .andThen(new L1Climb(climberSubsystem, swerveSubsystem)));

    return routine.cmd();
  }

  public Command trialPath() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    AutoTrajectory moveLeft = routine.trajectory("MoveLeft.traj");
    AutoTrajectory moveRight = routine.trajectory("MoveRight.traj");

    routine
        .active()
        .onTrue(moveLeft.resetOdometry().andThen(moveLeft.cmd()).andThen(moveRight.cmd()));

    return routine.cmd();
  }

  public Command trialPathTwo(
      Maneuver selectedManeuver,
      Intake selectedIntake,
      ShootPos selectedShootPos,
      ClimbPos selectedClimbPos) {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory maneuver = maneuver(routine, selectedManeuver);
    AutoTrajectory intake = intake(routine, selectedIntake);
    AutoTrajectory shootPos = shoot(routine, selectedShootPos);
    AutoTrajectory climbPos = climb(routine, selectedClimbPos);

    routine
        .active()
        .onTrue(
            (maneuver != null ? maneuver.resetOdometry() : Commands.none())
                .andThen(getPathCommandSafely(maneuver))
                .andThen(getPathCommandSafely(intake))
                .andThen(getPathCommandSafely(shootPos))
                .andThen(getPathCommandSafely(climbPos)));

    return routine.cmd();
  }

  public Command getPathCommandSafely(AutoTrajectory traj) {
    return traj != null ? traj.cmd() : Commands.none();
  }

  public void addCommandstoAutoChooser() {
    autoChooser.addCmd(
        "Red Pedri - Left Side",
        () ->
            RedPedri(
                null,
                Constants.Swerve.Auto.Intake.RedLeftIntakeL,
                Constants.Swerve.Auto.ShootPos.RedLeftShoot,
                Constants.Swerve.Auto.ClimbPos.RedLeftClimb));

    autoChooser.addCmd("Trial Path", () -> trialPath());

    autoChooser.addCmd(
        "Trial Path Two",
        () -> trialPathTwo(Constants.Swerve.Auto.Maneuver.RedRightManeuverR, null, null, null));
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
