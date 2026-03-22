package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private AutoTrajectory shoot(AutoRoutine routine, ShootPos type) {
    if (type == null) return null;

    AutoTrajectory traj = routine.trajectory(type + ".traj");
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

  public Command driveForward(double time) {
    return Commands.run(
            () -> swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(5, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time);
    // .andThen(
    //     () ->
    //         swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveBackward(double time) {
    return Commands.run(
            () ->
                swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(-5, 0, 0), new Feedforwards(4)),
            swerveSubsystem)
        .withTimeout(time);
    // .andThen(
    //     () ->
    //         swerveSubsystem.applyFieldSpeeds(new ChassisSpeeds(0, 0, 0), new Feedforwards(4)));
  }

  public Command driveToBumpAfterIntake(BooleanSupplier isRedSide) {
    // return new IntakeToBumpDTP(swerveSubsystem, isRedSide).withTimeout(0.5);
    return new IntakeToBumpDTP(swerveSubsystem, isRedSide);
  }

  // Auto paths without climb
  public AutoRoutine PedriMidRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriMidLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

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
        .onTrue(Commands.sequence(backward, intakeToShoot1.resetOdometry(), intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(Commands.sequence(backward2, intakeToShoot2.resetOdometry(), intakeToShoot2.cmd()));

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
                driveToBumpAfterIntake(redSide),
                backward,
                intakeToShoot1.resetOdometry(),
                intakeToShoot1.cmd()));

    intakeToShoot1.done().onTrue(Commands.sequence(returnBasicShoot(redSide), shootToBump.cmd()));

    shootToBump.done().onTrue(Commands.sequence(forward2, intake2.resetOdometry(), intake2.cmd()));

    intake2
        .done()
        .onTrue(Commands.sequence(backward2, intakeToShoot2.resetOdometry(), intakeToShoot2.cmd()));

    intakeToShoot2.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriDepotRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotL);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), depotIntake.cmd()));
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriDepotLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotR);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), depotIntake.cmd()));
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriOutpostRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostL);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), outpostIntake.cmd()));

    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine PedriOutpostLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    Command forward =
        redSide.getAsBoolean()
            ? driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS)
            : driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_FORWARDS);
    Command backward =
        redSide.getAsBoolean()
            ? driveForward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS)
            : driveBackward(Constants.Swerve.Auto.TIME_FOR_BUMP_BACKWARDS);

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostR);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine.active().onTrue(Commands.sequence(forward, intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(Commands.sequence(backward, intakeToShoot.resetOdometry(), intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), outpostIntake.cmd()));
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeOutpostShort() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostStart);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine.active().onTrue(Commands.sequence(outpostIntake.resetOdometry(), outpostIntake.cmd()));

    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeOutpostLong() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostStart);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);
    AutoTrajectory rightSweep = miscPaths(routine, Constants.Swerve.Auto.MiscPaths.RightSweep);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotL);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine.active().onTrue(Commands.sequence(outpostIntake.resetOdometry(), outpostIntake.cmd()));

    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), rightSweep.cmd()));
    rightSweep.done().onTrue(depotIntake.cmd());
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeDepotShort() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotStart);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));

    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine DrakeDepotLong() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotStart);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);
    AutoTrajectory leftSweep = miscPaths(routine, Constants.Swerve.Auto.MiscPaths.LeftSweep);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostR);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));

    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(Commands.sequence(returnBasicShoot(redSide), leftSweep.cmd()));
    leftSweep.done().onTrue(outpostIntake.cmd());
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  public AutoRoutine Nike() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory move = miscPaths(routine, Constants.Swerve.Auto.MiscPaths.Nike);

    routine.active().onTrue(Commands.sequence(move.resetOdometry(), move.cmd()));

    move.done().onTrue(returnBasicShoot(redSide));

    return routine;
  }

  // Add paths to chooser
  public void addCommandstoAutoChooser() {
    autoChooser.addRoutine("Left long depot", () -> PedriDepotLeft());
    autoChooser.addRoutine("Right Long depot", () -> PedriDepotRight());
    autoChooser.addRoutine("Left long outpost", () -> PedriOutpostLeft());
    autoChooser.addRoutine("Right long outpost", () -> PedriOutpostRight());

    autoChooser.addRoutine("Right two cycle", () -> PedriShortRight());
    autoChooser.addRoutine("Left two cycle", () -> PedriShortLeft());

    autoChooser.addRoutine("Right long", () -> PedriMidRight());
    autoChooser.addRoutine("Left long", () -> PedriMidLeft());

    autoChooser.addRoutine("Outpost", () -> DrakeOutpostShort());
    autoChooser.addRoutine("Depot", () -> DrakeDepotShort());
    autoChooser.addRoutine("Outpost to depot", () -> DrakeOutpostLong());
    autoChooser.addRoutine("Depot to outpost", () -> DrakeDepotLong());

    autoChooser.addRoutine("We are genuinely the worst robot on the field", () -> Nike());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
