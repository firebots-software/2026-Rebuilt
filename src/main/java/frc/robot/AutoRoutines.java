package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Swerve.Auto.ClimbPos;
import frc.robot.Constants.Swerve.Auto.Depot;
import frc.robot.Constants.Swerve.Auto.Intake;
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

  // Trajectory loading and specific cmds
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

  public Command aimAtHub(BooleanSupplier isRedSide) {
    Command aim = Commands.run(
        () -> {
          double rot =
              swerveSubsystem.calculateRequiredRotationalRateWithFF(
                  isRedSide.getAsBoolean()
                      ? Constants.Landmarks.RED_HUB_2D.getTranslation()
                      : Constants.Landmarks.BLUE_HUB_2D.getTranslation());

          ChassisSpeeds currSpeeds = swerveSubsystem.getFieldSpeeds();
          currSpeeds.omegaRadiansPerSecond = rot;

          swerveSubsystem.applyOneFieldSpeeds(currSpeeds);
        }, swerveSubsystem).withTimeout(1);

    return Commands.sequence(aim.asProxy());
  }

  // Auto paths without climb
  public AutoRoutine PedriMidRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, forwardDTP), intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriMidLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, forwardDTP), intake.resetOdometry(), intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriShortRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweepShort);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostR);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP),
                intakeToShoot.resetOdometry(),
                intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(), outpostIntake.cmd()));
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriShortLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweepShort);
    AutoTrajectory intakeToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotL);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP),
                intakeToShoot.resetOdometry(),
                intakeToShoot.cmd()));

    intakeToShoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriDepotRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotL);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriDepotLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotR);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), depotIntake.cmd()));
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriOutpostRight() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RightIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostL);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), outpostIntake.cmd()));
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine PedriOutpostLeft() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.LeftIntakeSweep);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RightShoot);
    AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.OutpostR);
    AutoTrajectory outpostToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.OutpostToShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forwardDTP),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backDTP), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(returnBasicShoot(), outpostIntake.cmd()));
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot());

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
    outpostToShoot.done().onTrue(returnBasicShoot());

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
    outpostToShoot.done().onTrue(Commands.sequence(returnBasicShoot(), rightSweep.cmd()));
    rightSweep.done().onTrue(depotIntake.cmd());
    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine DrakeDepotShort() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.DepotStart);
    AutoTrajectory depotToShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.DepotToShoot);

    routine.active().onTrue(Commands.sequence(depotIntake.resetOdometry(), depotIntake.cmd()));

    depotIntake.done().onTrue(depotToShoot.cmd());
    depotToShoot.done().onTrue(returnBasicShoot());

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
    depotToShoot.done().onTrue(Commands.sequence(returnBasicShoot(), leftSweep.cmd()));
    leftSweep.done().onTrue(outpostIntake.cmd());
    outpostIntake
        .done()
        .onTrue(
            Commands.sequence(
                new WaitCommand(Constants.Swerve.Auto.TIME_FOR_OUTPOST_INTAKE),
                outpostToShoot.cmd()));
    outpostToShoot.done().onTrue(returnBasicShoot());

    return routine;
  }

  public AutoRoutine Nike() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    routine.active().onTrue(returnBasicShoot());

    return routine;
  }

  // Paths for p2
  public AutoRoutine p2BumpForward() {
    AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

    BooleanSupplier forward = () -> !redSide.getAsBoolean();
    BooleanSupplier backward = () -> redSide.getAsBoolean();

    AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.p2Intake);
    AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.LeftShoot);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                intake.resetOdometry(),
                new BumpDTP(swerveSubsystem, forward),
                intake.resetOdometry(),
                intake.cmd()));

    intake
        .done()
        .onTrue(
            Commands.sequence(
                new BumpDTP(swerveSubsystem, backward), shoot.resetOdometry(), shoot.cmd()));

    shoot.done().onTrue(Commands.sequence(aimAtHub(redSide), ));

    return routine;
  }

  // Add paths to chooser
  public void addCommandstoAutoChooser() {
    // autoChooser.addRoutine("Depot (Left) Extreme", () -> PedriDepotLeft());
    // autoChooser.addRoutine("Depot (Right) Extreme", () -> PedriDepotRight());
    // autoChooser.addRoutine("Outpost (Left) Extreme", () -> PedriOutpostLeft());
    // autoChooser.addRoutine("Outpost (Right) Extreme", () -> PedriOutpostRight());

    // autoChooser.addRoutine("Right Okay", () -> PedriShortRight());
    // autoChooser.addRoutine("Left Okay", () -> PedriShortLeft());

    // autoChooser.addRoutine("Right Lame", () -> PedriMidRight());
    // autoChooser.addRoutine("Left Lame", () -> PedriMidLeft());

    // autoChooser.addRoutine("DrakeOutpostShort", () -> DrakeOutpostShort());
    // autoChooser.addRoutine("DrakeOutpostLong", () -> DrakeOutpostLong());
    // autoChooser.addRoutine("DrakeDepotShort", () -> DrakeDepotShort());
    // autoChooser.addRoutine("DrakeDepotLong", () -> DrakeDepotLong());

    // autoChooser.addRoutine("We are genuinely the worst robot on the pitch", () -> Nike());

    autoChooser.addRoutine("Bump forward, one cycle", () -> p2BumpForward());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }
}
