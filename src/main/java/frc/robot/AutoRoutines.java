package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final AutoRoutine routine;

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

    routine = autoFactory.newRoutine("CristianoRonaldo.chor");
  }

  private AutoTrajectory maneuver(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftManeuverL":
        traj = routine.trajectory("LeftManeuverL.traj");
        break;
      case "LeftManeuverR":
        traj = routine.trajectory("LeftManeuverR.traj");
        break;

      case "RightManeuverL":
        traj = routine.trajectory("RightManeuverL.traj");
        break;
      case "RightManeuverR":
        traj = routine.trajectory("RightManeuverR.traj");
        break;

      default:
        traj = null;
        break;
    }

    return traj;
  }

  private AutoTrajectory intake(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftIntakeL":
        traj = routine.trajectory("LeftIntakeL.traj");
        break;
      case "LeftIntakeM":
        traj = routine.trajectory("LeftIntakeM.traj");
        break;
      case "LeftIntakeR":
        traj = routine.trajectory("LeftIntakeR.traj");
        break;
      case "LeftIntakeML":
        traj = routine.trajectory("LeftIntakeML.traj");
        break;
      case "LeftIntakeMR":
        traj = routine.trajectory("LeftIntakeMR.traj");
        break;

      case "RightIntakeL":
        traj = routine.trajectory("RightIntakeL.traj");
        break;
      case "RightIntakeM":
        traj = routine.trajectory("RightIntakeM.traj");
        break;
      case "RightIntakeR":
        traj = routine.trajectory("RightIntakeR.traj");
        break;
      case "RightIntakeML":
        traj = routine.trajectory("RightIntakeML.traj");
        break;
      case "RightIntakeMR":
        traj = routine.trajectory("RightIntakeMR.traj");
        break;

      default:
        traj = null;
        break;
    }

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
      traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
    }

    return traj;
  }

  private AutoTrajectory shootPositioning(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftShootPositioning":
        traj = routine.trajectory("LeftShootPositioning.traj");
        break;

      case "RightShootPositioning":
        traj = routine.trajectory("RightShootPositioning.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  private AutoTrajectory climbPositioning(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftClimbPositioning":
        traj = routine.trajectory("LeftClimbPositioning.traj");
        break;

      case "RightClimbPositioning":
        traj = routine.trajectory("RightClimbPositioning.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  public Command Pedri(String maneuverType, String intakeType, String shootType, String climbType) {
    AutoTrajectory maneuver = maneuver(maneuverType);
    AutoTrajectory intake = intake(intakeType);
    AutoTrajectory shootPositioning = shootPositioning(shootType);
    AutoTrajectory climbPositioning = climbPositioning(climbType);

    // add proper dtp
    routine
        .active()
        .onTrue(
            (maneuver != null ? maneuver.resetOdometry() : Commands.none())
                .andThen(maneuver != null ? maneuver.cmd() : Commands.none())
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(intake != null ? intake.cmd() : Commands.none())
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(shootPositioning != null ? shootPositioning.cmd() : Commands.none())
                .andThen(new Shoot(lebronShooterSubsystem, intakeSubsystem, hopperSubsystem))
                .andThen(climbPositioning != null ? climbPositioning.cmd() : Commands.none())
                .andThen(new L1Climb(climberSubsystem, swerveSubsystem)));

    return routine.cmd();
  }
}
