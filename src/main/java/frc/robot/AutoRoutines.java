package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRoutines {
  private final AutoFactory factory;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem lebron;
  private final HopperSubsystem hopper;
  private final CommandSwerveDrivetrain swerve;

  public AutoRoutines(
      AutoFactory factory,
      IntakeSubsystem intake,
      ShooterSubsystem lebron,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerve) {
    this.factory = factory;
    this.intake = intake;
    this.lebron = lebron;
    this.hopper = hopper;
    this.swerve = swerve;
  }

  private AutoTrajectory maneuver(AutoRoutine routine, String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftManeuverL":
        traj = routine.trajectory("LeftManeuverL");
        break;
      case "LeftManeuverR":
        traj = routine.trajectory("LeftManeuverR");
        break;

      case "RightManeuverL":
        traj = routine.trajectory("RightManeuverL");
        break;
      case "RightManeuverR":
        traj = routine.trajectory("RightManeuverR");
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
        traj = routine.trajectory("LeftIntakeL");
        break;
      case "LeftIntakeM":
        traj = routine.trajectory("LeftIntakeM");
        break;
      case "LeftIntakeR":
        traj = routine.trajectory("LeftIntakeR");
        break;
      case "LeftIntakeML":
        traj = routine.trajectory("LeftIntakeML");
        break;
      case "LeftIntakeMR":
        traj = routine.trajectory("LeftIntakeMR");
        break;

      case "RightIntakeL":
        traj = routine.trajectory("RightIntakeL");
        break;
      case "RightIntakeM":
        traj = routine.trajectory("RightIntakeM");
        break;
      case "RightIntakeR":
        traj = routine.trajectory("RightIntakeR");
        break;
      case "RightIntakeML":
        traj = routine.trajectory("RightIntakeML");
        break;
      case "RightIntakeMR":
        traj = routine.trajectory("RightIntakeMR");
        break;

      default:
        traj = null;
        break;
    }

    return traj;
  }

  private AutoTrajectory shoot(AutoRoutine routine, String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftShoot":
        traj = routine.trajectory("LeftShoot");
        break;

      case "RightShoot":
        traj = routine.trajectory("RightShoot");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  public Command Pedri(String maneuverType, String intakeType, String shootType) {
    AutoRoutine routine = factory.newRoutine("DrakeFlexible");

    AutoTrajectory maneuver = maneuver(routine, maneuverType);
    AutoTrajectory intake = intake(routine, intakeType);
    AutoTrajectory shoot = shoot(routine, shootType);

    // add dtp length
    routine
        .active()
        .onTrue(
            maneuver
                .resetOdometry()
                .andThen(maneuver.cmd())
                .andThen(new DriveToPose(swerve))
                .andThen(intake.cmd())
                .andThen(new DriveToPose(swerve))
                .andThen(shoot.cmd()));

    return routine.cmd();
  }
}
