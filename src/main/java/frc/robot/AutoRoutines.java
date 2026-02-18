package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final AutoFactory factory;
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
    this.factory = factory;
    this.intakeSubsystem = intake;
    this.lebronShooterSubsystem = lebron;
    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    this.climberSubsystem = climber;
  }

  private AutoTrajectory maneuver(AutoRoutine routine, String name) {
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

  private AutoTrajectory intake(AutoRoutine routine, String name) {
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

    traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

    return traj;
  }

  private AutoTrajectory shootPositioning(AutoRoutine routine, String name) {
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

  private AutoTrajectory climbPositioning(AutoRoutine routine, String name) {
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
    AutoRoutine routine = factory.newRoutine("DrakeFlexible");

    AutoTrajectory maneuver = maneuver(routine, maneuverType);
    AutoTrajectory intake = intake(routine, intakeType);
    AutoTrajectory shootPositioning = shootPositioning(routine, shootType);
    AutoTrajectory climbPositioning = climbPositioning(routine, climbType);

    // add dtp length
    routine
        .active()
        .onTrue(
            maneuver
                .resetOdometry()
                .andThen(maneuver.cmd())
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(intake.cmd())
                .andThen(new DriveToPose(swerveSubsystem))
                .andThen(shootPositioning.cmd())
                .andThen(new Shoot(lebronShooterSubsystem, intakeSubsystem, hopperSubsystem))
                .andThen(climbPositioning.cmd())
                .andThen(new L1Climb(climberSubsystem, swerveSubsystem)));

    return routine.cmd();
  }
}
