package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Swerve.Auto.ClimbPos;
import frc.robot.Constants.Swerve.Auto.Depot;
import frc.robot.Constants.Swerve.Auto.Intake;
import frc.robot.Constants.Swerve.Auto.Maneuver;
import frc.robot.Constants.Swerve.Auto.MiscPaths;
import frc.robot.Constants.Swerve.Auto.Outpost;
import frc.robot.Constants.Swerve.Auto.ShootPos;
import frc.robot.commandGroups.BumpDTP;
import frc.robot.commandGroups.ExtendIntake;
import frc.robot.commandGroups.RetractIntake;
import frc.robot.commandGroups.ShootBasicRetract;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.MiscUtils;
import java.util.function.BooleanSupplier;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem lebronShooterSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final CommandSwerveDrivetrain swerveSubsystem;
  private final BooleanSupplier redSide;
  private final BooleanSupplier forwardDTP;
  private final BooleanSupplier backDTP;

  // private final ClimberSubsystem climberSubsystem;

  // i will figure out alliance side and add supplier to this

  public AutoRoutines(
      IntakeSubsystem intake,
      ShooterSubsystem lebron,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerve,
      ClimberSubsystem climber,
      BooleanSupplier redSide) {
    this.intakeSubsystem = intake;
    this.lebronShooterSubsystem = lebron;
    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    // this.climberSubsystem = climber;
    this.redSide = redSide;
    forwardDTP = () -> !redSide.getAsBoolean();
    backDTP = () -> redSide.getAsBoolean();

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

  private AutoTrajectory depot(AutoRoutine routine, Depot type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
      traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
    }

    return traj;
  }

  private AutoTrajectory outpost(AutoRoutine routine, Outpost type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    return traj;
  }

  private AutoTrajectory miscPaths(AutoRoutine routine, MiscPaths type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    return traj;
  }

  public AutoRoutine test() {
    BooleanSupplier forwardSupplier = () -> !RobotContainer.setAlliance();
    BooleanSupplier backSupplier = () -> RobotContainer.setAlliance();

    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.p2Intake);

    routine.active().onTrue(Commands.sequence(intake.resetOdometry(), new BumpDTP(swerveSubsystem, forwardSupplier), intake.resetOdometry(), intake.cmd()));

    intake.done().onTrue(Commands.sequence(new BumpDTP(swerveSubsystem, backSupplier)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardSupplier),
                intake.resetOdometry(),
                intake.cmd()));

    intake.done().onTrue(Commands.sequence(new BumpDTP(swerveSubsystem, backSupplier)));

    return routine;
  }

  public Command returnBasicShoot() {
    Command shoot =
        new ShootBasicRetract(
                () ->
                    MiscUtils.computeShootingSpeed(
                        MiscUtils.getDistanceToHub(redSide, swerveSubsystem)),
                () -> lebronShooterSubsystem.isAtSpeed(),
                lebronShooterSubsystem,
                intakeSubsystem,
                hopperSubsystem)
            .withTimeout(4);

    return Commands.sequence(shoot.asProxy());
  }

  // // this may not be needed, but is good to have
  // public Command getPathCommandSafely(AutoTrajectory traj) {
  //   return traj != null ? traj.cmd() : Commands.none();
  // }

  // // this may not be needed, but is good to have
  // public Command resetPathOdometrySafely(AutoTrajectory traj) {
  //   return traj != null ? traj.resetOdometry() : Commands.none();
  // }

  // if first path is null and odo is not reset, should i add smth to reset next
  // path or is that OD
  public void addCommandstoAutoChooser() {
    autoChooser.addRoutine("go right", () -> test());

  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
