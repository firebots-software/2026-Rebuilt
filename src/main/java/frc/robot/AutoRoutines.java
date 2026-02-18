package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  public static enum maneuverType {
    RLML("RedLeftManeuverL"), 
    RLMR("RedLeftManeuverR"),
    RRML("RedRightManeuverL"),
    RRMR("RedRightManeuverR"),
    BLML("BlueLeftManeuverL"),
    BLMR("BlueLeftManeuverR"),
    BRML("BlueRightManeuverL"),
    BRMR("BlueRightManeuverR");


    private final String value;

    maneuverType(String value) {
      this.value = value;
    }

    public String getValue() {
      return value;
    }
  }


  private AutoTrajectory maneuver(maneuverType type) {
    AutoTrajectory traj;

    switch (type) {
      case RLML:
        traj = routine.trajectory("RedLeftManeuverL.traj");
        break;
      case RLMR:
        traj = routine.trajectory("RedLeftManeuverR.traj");
        break;

      case RRML:
        traj = routine.trajectory("RedRightManeuverL.traj");
        break;
      case RRMR:
        traj = routine.trajectory("RedRightManeuverR.traj");
        break;

      case BLML:
        traj = routine.trajectory("BlueLeftManeuverL.traj");
        break;
      case BLMR:
        traj = routine.trajectory("BlueLeftManeuverR.traj");
        break;

      case BRML:
        traj = routine.trajectory("BlueRightManeuverL.traj");
        break;
      case BRMR:
        traj = routine.trajectory("BlueRightManeuverR.traj");
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
      case "RedLeftIntakeL":
        traj = routine.trajectory("RedLeftIntakeL.traj");
        break;
      case "RedLeftIntakeM":
        traj = routine.trajectory("RedLeftIntakeM.traj");
        break;
      case "RedLeftIntakeR":
        traj = routine.trajectory("RedLeftIntakeR.traj");
        break;
      case "RedLeftIntakeML":
        traj = routine.trajectory("RedLeftIntakeML.traj");
        break;
      case "RedLeftIntakeMR":
        traj = routine.trajectory("RedLeftIntakeMR.traj");
        break;

      case "RedRightIntakeL":
        traj = routine.trajectory("RedRightIntakeL.traj");
        break;
      case "RedRightIntakeM":
        traj = routine.trajectory("RedRightIntakeM.traj");
        break;
      case "RedRightIntakeR":
        traj = routine.trajectory("RedRightIntakeR.traj");
        break;
      case "RedRightIntakeML":
        traj = routine.trajectory("RedRightIntakeML.traj");
        break;
      case "RedRightIntakeMR":
        traj = routine.trajectory("RedRightIntakeMR.traj");
        break;

      case "BlueLeftIntakeL":
        traj = routine.trajectory("BlueLeftIntakeL.traj");
        break;
      case "BlueLeftIntakeM":
        traj = routine.trajectory("BlueLeftIntakeM.traj");
        break;
      case "BlueLeftIntakeR":
        traj = routine.trajectory("BlueLeftIntakeR.traj");
        break;
      case "BlueLeftIntakeML":
        traj = routine.trajectory("BlueLeftIntakeML.traj");
        break;
      case "BlueLeftIntakeMR":
        traj = routine.trajectory("BlueLeftIntakeMR.traj");
        break;

      case "BlueRightIntakeL":
        traj = routine.trajectory("BlueRightIntakeL.traj");
        break;
      case "BlueRightIntakeM":
        traj = routine.trajectory("BlueRightIntakeM.traj");
        break;
      case "BlueRightIntakeR":
        traj = routine.trajectory("BlueRightIntakeR.traj");
        break;
      case "BlueRightIntakeML":
        traj = routine.trajectory("BlueRightIntakeML.traj");
        break;
      case "BlueRightIntakeMR":
        traj = routine.trajectory("BlueRightIntakeMR.traj");
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

  private AutoTrajectory shoot(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "RedLeftShoot":
        traj = routine.trajectory("RedLeftShoot.traj");
        break;

      case "RedRightShoot":
        traj = routine.trajectory("RedRightShoot.traj");
        break;

      case "BlueLeftShoot":
        traj = routine.trajectory("BlueLeftShoot.traj");
        break;

      case "BlueRightShoot":
        traj = routine.trajectory("BlueRightShoot.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  private AutoTrajectory climb(String name) {
    AutoTrajectory traj;

    switch (name) {
      case "RedLeftClimb":
        traj = routine.trajectory("RedLeftClimb.traj");
        break;

      case "RedRightClimb":
        traj = routine.trajectory("RedRightClimb.traj");
        break;

      case "BlueLeftClimb":
        traj = routine.trajectory("BlueLeftClimb.traj");
        break;

      case "BlueRightClimb":
        traj = routine.trajectory("BlueRightClimb.traj");
        break;

      default:
        traj = null;
    }

    return traj;
  }

  public Command Pedri(maneuverType type, String intakeType, String shootType, String climbType) {
    AutoTrajectory maneuver = maneuver(type);
    AutoTrajectory intake = intake(intakeType);
    AutoTrajectory shootPositioning = shoot(shootType);
    AutoTrajectory climbPositioning = climb(climbType);

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

  public Command trialPath() {
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
}
