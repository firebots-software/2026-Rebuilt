package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Swerve.Auto.ClimbPos;
import frc.robot.Constants.Swerve.Auto.Intake;
import frc.robot.Constants.Swerve.Auto.Maneuver;
import frc.robot.Constants.Swerve.Auto.ShootPos;
import frc.robot.commandGroups.ClimbCommands.L1Climb;
import frc.robot.commandGroups.ExtendIntake;
import frc.robot.commandGroups.RetractIntake;
import frc.robot.commandGroups.Shoot;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem lebronShooterSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final CommandSwerveDrivetrain swerveSubsystem;
  private final ClimberSubsystem climberSubsystem;

  public AutoRoutines(
      AutoFactory factory,
      IntakeSubsystem intake,
      ShooterSubsystem lebron,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerve,
      ClimberSubsystem climber) {
    this.autoFactory = factory;
    this.intakeSubsystem = intake;
    this.lebronShooterSubsystem = lebron;
    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    this.climberSubsystem = climber;
  }

  private AutoTrajectory maneuver(AutoRoutine routine, Maneuver type) {
    if (type == null) return null;
    
    AutoTrajectory traj;

    switch (type) {
      case RedLeftManeuverL:
        traj = routine.trajectory("RedLeftManeuverL.traj");
        break;
      case RedLeftManeuverR:
        traj = routine.trajectory("RedLeftManeuverR.traj");
        break;

      case RedRightManeuverL:
        traj = routine.trajectory("RedRightManeuverL.traj");
        break;
      case RedRightManeuverR:
        traj = routine.trajectory("RedRightManeuverR.traj");
        break;

      case BlueLeftManeuverL:
        traj = routine.trajectory("BlueLeftManeuverL.traj");
        break;
      case BlueLeftManeuverR:
        traj = routine.trajectory("BlueLeftManeuverR.traj");
        break;

      case BlueRightManeuverL:
        traj = routine.trajectory("BlueRightManeuverL.traj");
        break;
      case BlueRightManeuverR:
        traj = routine.trajectory("BlueRightManeuverR.traj");
        break;

      default:
        traj = null;
        break;
    }

    return traj;
  }

  private AutoTrajectory intake(AutoRoutine routine, Intake type) {
    if (type == null) return null;
    
    AutoTrajectory traj;

    switch (type) {
      case RedLeftIntakeL:
        traj = routine.trajectory("RedLeftIntakeL.traj");
        break;
      case RedLeftIntakeM:
        traj = routine.trajectory("RedLeftIntakeM.traj");
        break;
      case RedLeftIntakeR:
        traj = routine.trajectory("RedLeftIntakeR.traj");
        break;
      case RedLeftIntakeML:
        traj = routine.trajectory("RedLeftIntakeML.traj");
        break;
      case RedLeftIntakeMR:
        traj = routine.trajectory("RedLeftIntakeMR.traj");
        break;

      case RedRightIntakeL:
        traj = routine.trajectory("RedRightIntakeL.traj");
        break;
      case RedRightIntakeM:
        traj = routine.trajectory("RedRightIntakeM.traj");
        break;
      case RedRightIntakeR:
        traj = routine.trajectory("RedRightIntakeR.traj");
        break;
      case RedRightIntakeML:
        traj = routine.trajectory("RedRightIntakeML.traj");
        break;
      case RedRightIntakeMR:
        traj = routine.trajectory("RedRightIntakeMR.traj");
        break;

      case BlueLeftIntakeL:
        traj = routine.trajectory("BlueLeftIntakeL.traj");
        break;
      case BlueLeftIntakeM:
        traj = routine.trajectory("BlueLeftIntakeM.traj");
        break;
      case BlueLeftIntakeR:
        traj = routine.trajectory("BlueLeftIntakeR.traj");
        break;
      case BlueLeftIntakeML:
        traj = routine.trajectory("BlueLeftIntakeML.traj");
        break;
      case BlueLeftIntakeMR:
        traj = routine.trajectory("BlueLeftIntakeMR.traj");
        break;

      case BlueRightIntakeL:
        traj = routine.trajectory("BlueRightIntakeL.traj");
        break;
      case BlueRightIntakeM:
        traj = routine.trajectory("BlueRightIntakeM.traj");
        break;
      case BlueRightIntakeR:
        traj = routine.trajectory("BlueRightIntakeR.traj");
        break;
      case BlueRightIntakeML:
        traj = routine.trajectory("BlueRightIntakeML.traj");
        break;
      case BlueRightIntakeMR:
        traj = routine.trajectory("BlueRightIntakeMR.traj");
        break;

      default:
        traj = null;
        break;
    }

    traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

    return traj;
  }

  private AutoTrajectory shoot(AutoRoutine routine, ShootPos type) {
    if (type == null) return null;
    
    AutoTrajectory traj;

    switch (type) {
      case RedLeftShoot:
        traj = routine.trajectory("RedLeftShoot.traj");
        break;

      case RedRightShoot:
        traj = routine.trajectory("RedRightShoot.traj");
        break;

      case BlueLeftShoot:
        traj = routine.trajectory("BlueLeftShoot.traj");
        break;

      case BlueRightShoot:
        traj = routine.trajectory("BlueRightShoot.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  private AutoTrajectory climb(AutoRoutine routine, ClimbPos type) {
    if (type == null) return null;
    
    AutoTrajectory traj;

    switch (type) {
      case RedLeftClimb:
        traj = routine.trajectory("RedLeftClimb.traj");
        break;

      case RedRightClimb:
        traj = routine.trajectory("RedRightClimb.traj");
        break;

      case BlueLeftClimb:
        traj = routine.trajectory("BlueLeftClimb.traj");
        break;

      case BlueRightClimb:
        traj = routine.trajectory("BlueRightClimb.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  public Command Pedri(
      Maneuver maneuverType, Intake intakeType, ShootPos shootType, ClimbPos climbType) {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    
    AutoTrajectory maneuver = maneuver(routine, maneuverType);
    AutoTrajectory intake = intake(routine, intakeType);
    AutoTrajectory shootPositioning = shoot(routine, shootType);
    AutoTrajectory climbPositioning = climb(routine, climbType);

    // add proper dtp
    routine
        .active()
        .onTrue(
            (maneuver != null ? maneuver.resetOdometry() : Commands.none())
                .andThen(getPathCommandSafely(maneuver))
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(getPathCommandSafely(intake))
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(getPathCommandSafely(shootPositioning))
                .andThen(new Shoot(lebronShooterSubsystem, intakeSubsystem, hopperSubsystem))
                .andThen(getPathCommandSafely(climbPositioning))
                .andThen(new L1Climb(climberSubsystem, swerveSubsystem)));

    return routine.cmd();
  }

  public Command trialPath() {
    AutoRoutine routine= autoFactory.newRoutine("CristianoRonaldo.chor");
    AutoTrajectory moveLeft = routine.trajectory("MoveLeft.traj");
    AutoTrajectory moveRight = routine.trajectory("MoveRight.traj");

    moveLeft.atTime("Log").onTrue(new InstantCommand(() -> DogLog.log("Reach target", true)));

    routine
        .active()
        .onTrue(
            moveLeft
                .resetOdometry()
                .andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> DogLog.log("start time", true)), moveLeft.cmd()))
                .andThen(moveRight.cmd()));

    return routine.cmd();
  }

  public Command getPathCommandSafely(AutoTrajectory traj) {
    return traj != null ? traj.cmd() : Commands.none();
  }
}
