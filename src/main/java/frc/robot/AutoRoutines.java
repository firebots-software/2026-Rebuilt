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

  // public Command doneWeek0Auto() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory moveLeft = routine.trajectory("MoveLeftWithMarker");
  //   AutoTrajectory moveRight = routine.trajectory("MoveRightWithMarker");

  //   routine.active().onTrue(Commands.sequence(moveLeft.resetOdometry(), moveLeft.cmd()));

  //   moveLeft.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   moveRight.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   // When the trajectory is done, start the next trajectory
  //   moveLeft.done().onTrue(moveRight.cmd());

  //   return routine.cmd();
  // }

  // public Command doneWeek0AutoWithShoot() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory moveLeft = routine.trajectory("MoveLeftWithMarker");
  //   AutoTrajectory moveRight = routine.trajectory("MoveRightWithMarker");

  //   routine.active().onTrue(Commands.sequence(moveLeft.resetOdometry(), moveLeft.cmd()));

  //   moveLeft.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   moveRight.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   // When the trajectory is done, start the next trajectory
  //   moveLeft.done().onTrue(moveRight.cmd());

  //   moveRight.done().onTrue(returnBasicShoot());

  //   return routine.cmd();
  // }

  // public AutoRoutine RedPedriDepotR() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotL);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   shoot.done().onTrue(depotIntake.cmd());
  //   depotIntake.done().onTrue(depotShoot.cmd());
  //   depotShoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public AutoRoutine RedPedriDepotL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotR);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   shoot.done().onTrue(depotIntake.cmd());
  //   depotIntake.done().onTrue(depotShoot.cmd());
  //   depotShoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public Command RedPedriOutpostL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
  //   AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
  //   AutoTrajectory outpostShoot = shoot(routine,
  // Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           new BumpDTP(swerveSubsystem, () -> true)
  //               .andThen(resetPathOdometrySafely(intake))
  //               .andThen(getPathCommandSafely(intake))
  //               .andThen(new BumpDTP(swerveSubsystem, () -> false))
  //               .andThen(resetPathOdometrySafely(shoot))
  //               .andThen(getPathCommandSafely(shoot))
  //               .andThen(returnBasicShoot())
  //               .andThen(getPathCommandSafely(outpostIntake))
  //               .andThen(new WaitCommand(3))
  //               .andThen(getPathCommandSafely(outpostShoot))
  //               .andThen(returnBasicShoot()));

  //   return routine.cmd();
  // }

  // public Command RedPedriOutpostR() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
  //   AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostL);
  //   AutoTrajectory outpostShoot = shoot(routine,
  // Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           new BumpDTP(swerveSubsystem, () -> true)
  //               .andThen(resetPathOdometrySafely(intake))
  //               .andThen(getPathCommandSafely(intake))
  //               .andThen(new BumpDTP(swerveSubsystem, () -> false))
  //               .andThen(resetPathOdometrySafely(shoot))
  //               .andThen(getPathCommandSafely(shoot))
  //               .andThen(returnBasicShoot())
  //               .andThen(getPathCommandSafely(outpostIntake))
  //               .andThen(new WaitCommand(3))
  //               .andThen(getPathCommandSafely(outpostShoot))
  //               .andThen(returnBasicShoot()));

  //   return routine.cmd();
  // }

  // public AutoRoutine RedPedriMidR() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public AutoRoutine RedPedriMidL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public Command RedPedriShortL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeShort);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotL);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           new BumpDTP(swerveSubsystem, () -> true)
  //               .andThen(resetPathOdometrySafely(intake))
  //               .andThen(getPathCommandSafely(intake))
  //               .andThen(new BumpDTP(swerveSubsystem, () -> false))
  //               .andThen(resetPathOdometrySafely(shoot))
  //               .andThen(getPathCommandSafely(shoot))
  //               .andThen(returnBasicShoot())
  //               .andThen(getPathCommandSafely(depotIntake))
  //               .andThen(getPathCommandSafely(depotShoot))
  //               .andThen(returnBasicShoot()));

  //   return routine.cmd();
  // }

  // public Command RedPedriShortR() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeShort);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
  //   AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
  //   AutoTrajectory outpostShoot = shoot(routine,
  // Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           new BumpDTP(swerveSubsystem, () -> true)
  //               .andThen(resetPathOdometrySafely(intake))
  //               .andThen(getPathCommandSafely(intake))
  //               .andThen(new BumpDTP(swerveSubsystem, () -> false))
  //               .andThen(resetPathOdometrySafely(shoot))
  //               .andThen(getPathCommandSafely(shoot))
  //               .andThen(returnBasicShoot())
  //               .andThen(getPathCommandSafely(outpostIntake))
  //               .andThen(new WaitCommand(3))
  //               .andThen(getPathCommandSafely(outpostShoot)));

  //   return routine.cmd();
  // }

  // public Command RedDrakeOutpostShort() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

  //   routine.active().onTrue(getPathCommandSafely(intake).andThen(getPathCommandSafely(shoot)));

  //   return routine.cmd();
  // }

  // public Command RedDrakeOutpostLong() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory outpostIntake = outpost(routine,
  // Constants.Swerve.Auto.Outpost.RedOutpostRDrake);
  //   AutoTrajectory outpostShoot = shoot(routine,
  // Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);
  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotR);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           getPathCommandSafely(outpostIntake)
  //               .andThen(getPathCommandSafely(outpostShoot))
  //               .andThen(returnBasicShoot())
  //               .andThen(getPathCommandSafely(depotIntake))
  //               .andThen(getPathCommandSafely(depotShoot)));
  //   return routine.cmd();
  // }

  // public Command RedDrakeDepotShort() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotRDrake);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(getPathCommandSafely(depotIntake).andThen(getPathCommandSafely(depotShoot)));

  //   return routine.cmd();
  // }

  // public Command RedDrakeDepotLong() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotRDrake);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);
  //   AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostL);
  //   AutoTrajectory outpostShoot =
  //       shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShootShort);

  //   routine
  //       .active()
  //       .onTrue(
  //           getPathCommandSafely(depotIntake)
  //               .andThen(getPathCommandSafely(depotShoot))
  //               .andThen(getPathCommandSafely(outpostIntake))
  //               .andThen(getPathCommandSafely(outpostShoot)));

  //   return routine.cmd();
  // }

  public AutoRoutine scrimOutpostSimple() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueRightIntakeSweepShort);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueRightShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.BlueDepotR);
    AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueDepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    intake.atTime("IntakeUp").onTrue(intakeSubsystem.intakeDefault());

    intake
        .done()
        .onTrue(
            Commands.sequence(
                shoot.resetOdometry(),
                new BumpDTP(swerveSubsystem, backDTP),
                shoot.resetOdometry(),
                shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotShoot.cmd());
    depotShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine scrimOutpostHard() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueRightIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueLeftShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.BlueDepotL);
    AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueDepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    intake.atTime("IntakeUp").onTrue(intakeSubsystem.intakeDefault());

    intake
        .done()
        .onTrue(
            Commands.sequence(
                shoot.resetOdometry(),
                new BumpDTP(swerveSubsystem, backDTP),
                shoot.resetOdometry(),
                shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotShoot.cmd());
    depotShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine scrimDepotSimple() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueLeftIntakeSweepShort);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueLeftShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.BlueDepotL);
    AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueDepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    intake.atTime("IntakeUp").onTrue(intakeSubsystem.intakeDefault());

    intake
        .done()
        .onTrue(
            Commands.sequence(
                shoot.resetOdometry(),
                new BumpDTP(swerveSubsystem, backDTP),
                shoot.resetOdometry(),
                shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotShoot.cmd());
    depotShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine scrimDepotHard() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueLeftIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueRightShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.BlueDepotR);
    AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueDepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
    intake.atTime("IntakeUp").onTrue(intakeSubsystem.intakeDefault());

    intake
        .done()
        .onTrue(
            Commands.sequence(
                shoot.resetOdometry(),
                new BumpDTP(swerveSubsystem, backDTP),
                shoot.resetOdometry(),
                shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotShoot.cmd());
    depotShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine test() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.p2Intake);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, forwardDTP), intake.resetOdometry(), intake.cmd()));

    intake.done().onTrue(Commands.sequence(new BumpDTP(swerveSubsystem, backDTP)));

    return routine;
  }

  public AutoRoutine testDTP() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");
    // AutoTrajectory right = routine.trajectory("GoRight.traj");

    // routine
    //     .active()
    //     .onTrue(
    //         Commands.sequence(
    //             right.resetOdometry(),
    //             new BumpDTP(swerveSubsystem, forwardSupplier),
    //             right.resetOdometry(),
    //             right.cmd()));
    routine.active().onTrue(Commands.sequence(new BumpDTP(swerveSubsystem, forwardDTP)));

    return routine;
  }

  // public AutoRoutine BluePedriDepotL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueLeftIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueRightShoot);
  //   AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.BlueDepotR);
  //   AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueDepotToShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));

  //   depotIntake.done().onTrue(depotShoot.cmd());
  //   depotShoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public AutoRoutine BluePedriMidR() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueRightIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueLeftShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

  // public AutoRoutine BluePedriMidL() {
  //   AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

  //   AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.BlueLeftIntakeSweep);
  //   AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.BlueRightShoot);

  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> true), intake.resetOdometry(), intake.cmd()));

  //   intake.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
  //   intake.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));

  //   intake
  //       .done()
  //       .onTrue(
  //           Commands.sequence(
  //               new BumpDTP(swerveSubsystem, () -> false), shoot.resetOdometry(), shoot.cmd()));

  //   shoot.done().onTrue(returnBasicShoot());

  //   return routine;
  // }

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

  // this may not be needed, but is good to have
  public Command getPathCommandSafely(AutoTrajectory traj) {
    return traj != null ? traj.cmd() : Commands.none();
  }

  // this may not be needed, but is good to have
  public Command resetPathOdometrySafely(AutoTrajectory traj) {
    return traj != null ? traj.resetOdometry() : Commands.none();
  }

  // if first path is null and odo is not reset, should i add smth to reset next
  // path or is that OD
  public void addCommandstoAutoChooser() {
    // autoChooser.addCmd("doneWeek0Path", () -> doneWeek0Auto());
    // autoChooser.addCmd("doneWeek0PathWithShoot", () -> doneWeek0AutoWithShoot());

    // autoChooser.addRoutine("Red Pedri - depot (right)", () -> RedPedriDepotR());
    // autoChooser.addRoutine("Red Pedri - depot (left)", () -> RedPedriDepotL());
    // autoChooser.addCmd("Red Pedri - outpost (right)", () -> RedPedriOutpostR());
    // autoChooser.addCmd("Red Pedri - outpost (left)", () -> RedPedriOutpostL());
    // autoChooser.addCmd("Red Pedri - short (right)", () -> RedPedriShortR());
    // autoChooser.addCmd("Red Pedri - short (left)", () -> RedPedriShortL());
    // autoChooser.addRoutine("Red Pedri - mid (right)", () -> RedPedriMidR());
    // autoChooser.addRoutine("Red Pedri - mid (left)", () -> RedPedriMidL());
    // autoChooser.addCmd("Red Drake - depot (long)", () -> RedDrakeDepotLong());
    // autoChooser.addCmd("Red Drake - depot (short)", () -> RedDrakeDepotShort());
    // autoChooser.addCmd("Red Drake - outpost (long)", () -> RedDrakeOutpostLong());
    // autoChooser.addCmd("Red Drake - outpost (short)", () -> RedDrakeOutpostShort());
    // autoChooser.addCmd("Nike", () -> Nike());

    // autoChooser.addRoutine("Red Pedri - depot (right)", () -> RedPedriDepotR());
    // autoChooser.addRoutine("Red Pedri - depot (left)", () -> RedPedriDepotL());
    // autoChooser.addRoutine("Red Pedri - mid (right)", () -> RedPedriMidR());
    // autoChooser.addRoutine("Red Pedri - mid (left)", () -> RedPedriMidL());
    // autoChooser.addRoutine("Blue Pedri - mid (right)", () -> BluePedriMidR());
    // autoChooser.addRoutine("Blue Pedri - mid (left)", () -> BluePedriMidL());

    autoChooser.addRoutine("Outpost Side Simple", () -> scrimOutpostSimple());
    autoChooser.addRoutine("Outpost Side Hard", () -> scrimOutpostHard());

    autoChooser.addRoutine("Depot Side Simple", () -> scrimDepotSimple());
    autoChooser.addRoutine("Depot Side Hard", () -> scrimDepotHard());

    autoChooser.addRoutine("go right", () -> test());
    autoChooser.addRoutine("dtp then right", () -> testDTP());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
