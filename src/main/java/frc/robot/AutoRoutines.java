package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Swerve.Auto.Depot;
import frc.robot.Constants.Swerve.Auto.Intake;
import frc.robot.Constants.Swerve.Auto.MiscPaths;
import frc.robot.Constants.Swerve.Auto.Outpost;
import frc.robot.Constants.Swerve.Auto.ShootPos;
import frc.robot.commandGroups.IntakeToBumpDTP;
import frc.robot.commandGroups.ShootCommandGroups.ShootWithAim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class AutoRoutines {
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem lebronShooterSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final CommandSwerveDrivetrain swerveSubsystem;
  private final BooleanSupplier redSide;

  public AutoRoutines(
      IntakeSubsystem intake,
      ShooterSubsystem lebron,
      HopperSubsystem hopper,
      CommandSwerveDrivetrain swerve,
      BooleanSupplier redSide) {
    this.intakeSubsystem = intake;
    this.lebronShooterSubsystem = lebron;
    this.hopperSubsystem = hopper;
    this.swerveSubsystem = swerve;
    this.redSide = redSide;

    autoFactory = swerveSubsystem.createAutoFactory();
    autoChooser = new AutoChooser();
    addCommandstoAutoChooser();
  }

  private AutoTrajectory shoot(AutoRoutine routine, ShootPos type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");
    return traj;
  }

  // Trajectory loading and specific cmds
  private AutoTrajectory intake(AutoRoutine routine, Intake type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(intakeSubsystem.intakeUntilInterruptedCommand());
      traj.atTime("IntakeUp").onTrue(intakeSubsystem.retractIntakeCommand());
    }
    return traj;
  }

  private AutoTrajectory depot(AutoRoutine routine, Depot type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(intakeSubsystem.intakeUntilInterruptedCommand());
      traj.atTime("IntakeUp").onTrue(intakeSubsystem.retractIntakeCommand());
    }
    return traj;
  }

  private AutoTrajectory outpost(AutoRoutine routine, Outpost type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");

    if (traj != null) {
      traj.atTime("IntakeDown").onTrue(intakeSubsystem.intakeUntilInterruptedCommand());
      traj.atTime("IntakeUp").onTrue(intakeSubsystem.retractIntakeCommand());
    }
    return traj;
  }

  private AutoTrajectory miscPaths(AutoRoutine routine, MiscPaths type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");
    return traj;
  }

  public Command returnBasicShoot(BooleanSupplier isRedSide) {
    Command shoot =
        new ShootWithAim(
                () -> 0.0,
                () -> 0.0,
                lebronShooterSubsystem,
                intakeSubsystem,
                hopperSubsystem,
                swerveSubsystem,
                isRedSide,
                () -> false)
            .withTimeout(4.0);

    return shoot;
  }

  public Command returnBasicShootLessTime(BooleanSupplier isRedSide) {
    Command shoot =
        new ShootWithAim(
                () -> 0.0,
                () -> 0.0,
                lebronShooterSubsystem,
                intakeSubsystem,
                hopperSubsystem,
                swerveSubsystem,
                isRedSide,
                () -> false)
            .withTimeout(2.5);

    return shoot;
  }

  public Command driveForward(double time) {
    return Commands.run(
            () -> swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(3, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time)
        .andThen(
            () ->
                swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveBackward(double time) {
    return Commands.run(
            () ->
                swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(-3, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time)
        .andThen(
            () ->
                swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveForwardSlower(double time) {
    return Commands.run(
            () ->
                swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(2.7, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time);
    // .andThen(
    //     () ->
    //         swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveBackwardSlower(double time) {
    return Commands.run(
            () ->
                swerveSubsystem.applyFieldSpeeds(
                    new ChassisSpeeds(-2.7, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time);
    // .andThen(
    //     () ->
    //         swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveToBumpAfterIntake(BooleanSupplier isRedSide, BooleanSupplier isLeftSide) {
    // return new IntakeToBumpDTP(swerveSubsystem, isRedSide).withTimeout(0.5);
    return new IntakeToBumpDTP(swerveSubsystem, isRedSide, isLeftSide);
  }

  // Middle
  public AutoRoutine Nike() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory move = miscPaths(routine, Constants.Swerve.Auto.MiscPaths.Nike);

    routine.active().onTrue(Commands.sequence(move.resetOdometry(), move.cmd()));

    move.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  // Sweeps
  public AutoRoutine PedriShortRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);
    Command forward2 =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward2 =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake1 = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweepShort);
    AutoTrajectory intakeToShoot1 = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory shootToBump =
        miscPaths(routine, Constants.Swerve.Auto.MiscPaths.RightShootToBump);
    AutoTrajectory intake2 = intake(routine, Constants.Swerve.Auto.Intake.RightSecondDip);
    AutoTrajectory intakeToShoot2 = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);

    routine.active().onTrue(Commands.sequence(forward, intake1.resetOdometry(), intake1.cmd()));

    intake1
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> false),
                backward,
                intakeToShoot1.resetOdometry(),
                intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> false),
                backward2,
                intakeToShoot2.resetOdometry(),
                intakeToShoot2.cmd()));

    intakeToShoot2.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriShortLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);
    Command forward2 =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward2 =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake1 = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweepShort);
    AutoTrajectory intakeToShoot1 = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory shootToBump =
        miscPaths(routine, Constants.Swerve.Auto.MiscPaths.LeftShootToBump);
    AutoTrajectory intake2 = intake(routine, Constants.Swerve.Auto.Intake.LeftSecondDip);
    AutoTrajectory intakeToShoot2 = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);

    routine.active().onTrue(Commands.sequence(forward, intake1.resetOdometry(), intake1.cmd()));

    intake1
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> true),
                backward,
                intakeToShoot1.resetOdometry(),
                intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> true),
                backward2,
                intakeToShoot2.resetOdometry(),
                intakeToShoot2.cmd()));

    intakeToShoot2.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine HubSweepRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER)
            : driveForwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);
    Command forward2 =
        redSide.getAsBoolean()
            ? driveBackwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER)
            : driveForwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER);
    Command backward2 =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake1 = intake(routine, Constants.Swerve.Auto.Intake.HubRight);
    AutoTrajectory intakeToShoot1 = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory shootToBump =
        miscPaths(routine, Constants.Swerve.Auto.MiscPaths.RightShootToBump);
    AutoTrajectory intake2 = intake(routine, Constants.Swerve.Auto.Intake.HubRight);
    AutoTrajectory intakeToShoot2 = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(Constants.Swerve.Auto.WAIT_TIME_FOR_ALLIANCE),
                forward,
                intake1.resetOdometry(),
                intake1.cmd()));

    intake1
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> false),
                backward,
                intakeToShoot1.resetOdometry(),
                intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> false),
                backward2,
                intakeToShoot2.resetOdometry(),
                intakeToShoot2.cmd()));

    intakeToShoot2.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine HubSweepLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER)
            : driveForwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);
    Command forward2 =
        redSide.getAsBoolean()
            ? driveBackwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER)
            : driveForwardSlower(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS_SLOWER);
    Command backward2 =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake1 = intake(routine, Constants.Swerve.Auto.Intake.HubLeft);
    AutoTrajectory intakeToShoot1 = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory shootToBump =
        miscPaths(routine, Constants.Swerve.Auto.MiscPaths.LeftShootToBump);
    AutoTrajectory intake2 = intake(routine, Constants.Swerve.Auto.Intake.HubLeft);
    AutoTrajectory intakeToShoot2 = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(Constants.Swerve.Auto.WAIT_TIME_FOR_ALLIANCE),
                forward,
                intake1.resetOdometry(),
                intake1.cmd()));

    intake1
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> true),
                backward,
                intakeToShoot1.resetOdometry(),
                intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(
            Commands.sequence(
                driveToBumpAfterIntake(redSide, () -> true),
                backward2,
                intakeToShoot2.resetOdometry(),
                intakeToShoot2.cmd()));

    intakeToShoot2.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  // //Outpost and Depot
  public AutoRoutine DrakeOutpostShort() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostStartPush);

    routine.active().onTrue(Commands.sequence(outpostIntake.resetOdometry(), outpostIntake.cmd()));

    outpostIntake.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeDepotShort() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotStart);

    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));

    depotIntake.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeOutpostLong() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostStart);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotSweep);
    Command shoot = returnBasicShootLessTime(redSide);
    Trigger shootIsFinished = new Trigger(() -> shoot.isFinished());

    // routine.active().onTrue(Commands.sequence(outpostIntake.resetOdometry(),
    // outpostIntake.cmd()));
    // outpostIntake.done().onTrue(Commands.sequence(returnBasicShoot(redSide).asProxy(),
    // depotIntake.cmd()));
    // depotIntake.done().onTrue(returnBasicShoot(redSide));
    routine.active().onTrue(Commands.sequence(outpostIntake.resetOdometry(), outpostIntake.cmd()));
    outpostIntake.done().onTrue(shoot);
    shootIsFinished.onTrue(depotIntake.cmd());
    depotIntake.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeDepotLong() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotStart);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostSweep);
    Command shoot = returnBasicShoot(redSide);
    Trigger shootIsFinished = new Trigger(() -> shoot.isFinished());

    // routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));
    // depotIntake.done().onTrue(Commands.sequence(returnBasicShoot(redSide).asProxy(),
    // outpostIntake.cmd()));
    // outpostIntake.done().onTrue(returnBasicShoot(redSide));
    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));
    depotIntake.done().onTrue(shoot);
    shootIsFinished.onTrue(outpostIntake.cmd());
    outpostIntake.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine anthony() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = miscPaths(routine, Constants.Swerve.Auto.MiscPaths.anthony);

    // routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));
    // depotIntake.done().onTrue(Commands.sequence(returnBasicShoot(redSide).asProxy(),
    // outpostIntake.cmd()));
    // outpostIntake.done().onTrue(returnBasicShoot(redSide));
    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));

    return routine;
  }

  // Add paths to chooser
  public void addCommandstoAutoChooser() {
    autoChooser.addRoutine("Right", () -> PedriShortRight());
    autoChooser.addRoutine("Left", () -> PedriShortLeft());
    autoChooser.addRoutine("Hub Sweep Left", () -> HubSweepLeft());
    autoChooser.addRoutine("Hub Sweep Right", () -> HubSweepRight());

    autoChooser.addRoutine("Just Outpost", () -> DrakeOutpostShort());
    autoChooser.addRoutine("Just Depot", () -> DrakeDepotShort());
    autoChooser.addRoutine("Outpost to Depot", () -> DrakeOutpostLong());
    autoChooser.addRoutine("Depot to Outpost", () -> DrakeDepotLong());

    autoChooser.addRoutine("We are genuinely the worst robot on the field", () -> Nike());
    autoChooser.addRoutine("bad word", () -> anthony());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
