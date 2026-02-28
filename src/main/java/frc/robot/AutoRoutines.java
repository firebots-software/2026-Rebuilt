// package frc.robot;

// import choreo.auto.AutoChooser;
// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commandGroups.BumpDTP;
// import frc.robot.commandGroups.ExtendIntake;
// import frc.robot.commandGroups.RetractIntake;
// import frc.robot.commandGroups.ShootBasic;
// import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.HopperSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.util.ChoreoTraj;
// import frc.robot.util.Targeting;

// public class AutoRoutines {
//   private final AutoFactory autoFactory;
//   private final AutoChooser autoChooser;
//   private final IntakeSubsystem intakeSubsystem;
//   private final ShooterSubsystem lebronShooterSubsystem;
//   private final HopperSubsystem hopperSubsystem;
//   private final CommandSwerveDrivetrain swerveSubsystem;

//   // private final ClimberSubsystem climberSubsystem;

//   // i will figure out alliance side and add supplier to this

//   public AutoRoutines(
//       IntakeSubsystem intake,
//       ShooterSubsystem lebron,
//       HopperSubsystem hopper,
//       CommandSwerveDrivetrain swerve,
//       ClimberSubsystem climber) {
//     this.intakeSubsystem = intake;
//     this.lebronShooterSubsystem = lebron;
//     this.hopperSubsystem = hopper;
//     this.swerveSubsystem = swerve;
//     // this.climberSubsystem = climber;

//     autoFactory = swerveSubsystem.createAutoFactory();

//     ChoreoTraj trajectories = new ChoreoTraj(null, null, 0, null, null)

//     autoChooser = new AutoChooser();
//     addCommandstoAutoChooser();
//   }

//   private AutoTrajectory maneuver(AutoRoutine routine, Maneuver type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     return traj;
//   }

//   private AutoTrajectory intake(AutoRoutine routine, Intake type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     if (traj != null) {
//       traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
//       traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
//     }

//     return traj;
//   }

//   private AutoTrajectory shoot(AutoRoutine routine, ShootPos type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     return traj;
//   }

//   private AutoTrajectory climb(AutoRoutine routine, ClimbPos type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     return traj;
//   }

//   private AutoTrajectory depot(AutoRoutine routine, Depot type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     if (traj != null) {
//       traj.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
//       traj.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
//     }

//     return traj;
//   }

//   private AutoTrajectory outpost(AutoRoutine routine, Outpost type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     return traj;
//   }

//   private AutoTrajectory miscPaths(AutoRoutine routine, MiscPaths type) {
//     if (type == null) return null;

//     AutoTrajectory traj = routine.trajectory(type + ".traj");

//     return traj;
//   }

//   public Command doneWeek0Auto() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory moveLeft = routine.trajectory("MoveLeftWithMarker");
//     AutoTrajectory moveRight = routine.trajectory("MoveRightWithMarker");
 
//     routine.active().onTrue(
//         Commands.sequence(
//             moveLeft.resetOdometry(),
//             moveLeft.cmd()
//         )
//     );

//     moveLeft.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
//     moveRight.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
   
//     // When the trajectory is done, start the next trajectory
//     moveLeft.done().onTrue(moveRight.cmd());

//     return routine.cmd();
//   }
  
//   public Command doneWeek0AutoWithShoot() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory moveLeft = routine.trajectory("MoveLeftWithMarker");
//     AutoTrajectory moveRight = routine.trajectory("MoveRightWithMarker");
 
//     routine.active().onTrue(
//         Commands.sequence(
//             moveLeft.resetOdometry(),
//             moveLeft.cmd()
//         )
//     );

//     moveLeft.atTime("IntakeDown").onTrue(new ExtendIntake(intakeSubsystem));
//     moveRight.atTime("IntakeUp").onTrue(new RetractIntake(intakeSubsystem));
   
//     // When the trajectory is done, start the next trajectory
//     moveLeft.done().onTrue(moveRight.cmd());

//     moveRight.done().onTrue(returnBasicShoot());

//     return routine.cmd();
//   }

//   public Command week0Auto() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory moveLeft = routine.trajectory("MoveLeft.traj");
//     AutoTrajectory moveRight = routine.trajectory("MoveRight.traj");

//     routine
//         .active()
//         .onTrue(
//             moveLeft
//                 .resetOdometry()
//                 .andThen(
//                     new ParallelCommandGroup(
//                         moveLeft.cmd(), intakeSubsystem.intakeUntilInterruptedCommand()))
//                 .andThen(
//                     new ParallelCommandGroup(
//                         moveRight.cmd(),
//                         intakeSubsystem.setArmToDegreesCommand(
//                             Constants.Intake.Arm.ARM_POS_RETRACTED))));

//     return routine.cmd();
//   }

//   public Command week0AutoWithCommands() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory moveLeft = routine.trajectory("MoveLeft.traj");
//     AutoTrajectory moveRight = routine.trajectory("MoveRight.traj");

//     routine
//         .active()
//         .onTrue(
//             moveLeft
//                 .resetOdometry()
//                 .andThen(
//                     new ParallelCommandGroup(
//                         moveLeft.cmd(), new ExtendIntake(intakeSubsystem)))
//                 .andThen(
//                     new ParallelCommandGroup(
//                         moveRight.cmd(),
//                         new RetractIntake(intakeSubsystem))));

//     return routine.cmd();
//   }

//   public Command RedPedriDepotR() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotL);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(depotIntake))
//                 .andThen(getPathCommandSafely(depotShoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriDepotL() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotR);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(depotIntake))
//                 .andThen(getPathCommandSafely(depotShoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriOutpostL() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
//     AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
//     AutoTrajectory outpostShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(outpostIntake))
//                 .andThen(new WaitCommand(3))
//                 .andThen(getPathCommandSafely(outpostShoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriOutpostR() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
//     AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostL);
//     AutoTrajectory outpostShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(outpostIntake))
//                 .andThen(new WaitCommand(3))
//                 .andThen(getPathCommandSafely(outpostShoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriMidR() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriMidL() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeSweep);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriShortL() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedLeftIntakeShort);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedLeftShoot);
//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotL);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(depotIntake))
//                 .andThen(getPathCommandSafely(depotShoot))
//                 .andThen(returnBasicShoot()));

//     return routine.cmd();
//   }

//   public Command RedPedriShortR() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = intake(routine, Constants.Swerve.Auto.Intake.RedRightIntakeShort);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedRightShoot);
//     AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
//     AutoTrajectory outpostShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

//     routine
//         .active()
//         .onTrue(
//             new BumpDTP(swerveSubsystem, () -> true)
//                 .andThen(resetPathOdometrySafely(intake))
//                 .andThen(getPathCommandSafely(intake))
//                 .andThen(new BumpDTP(swerveSubsystem, () -> false))
//                 .andThen(resetPathOdometrySafely(shoot))
//                 .andThen(getPathCommandSafely(shoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(outpostIntake))
//                 .andThen(new WaitCommand(3))
//                 .andThen(getPathCommandSafely(outpostShoot)));

//     return routine.cmd();
//   }

//   public Command RedDrakeOutpostShort() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory intake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostR);
//     AutoTrajectory shoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);

//     routine.active().onTrue(getPathCommandSafely(intake).andThen(getPathCommandSafely(shoot)));

//     return routine.cmd();
//   }

//   public Command RedDrakeOutpostLong() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostRDrake);
//     AutoTrajectory outpostShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShoot);
//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotR);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

//     routine
//         .active()
//         .onTrue(
//             getPathCommandSafely(outpostIntake)
//                 .andThen(getPathCommandSafely(outpostShoot))
//                 .andThen(returnBasicShoot())
//                 .andThen(getPathCommandSafely(depotIntake))
//                 .andThen(getPathCommandSafely(depotShoot)));
//     return routine.cmd();
//   }

//   public Command RedDrakeDepotShort() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotRDrake);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);

//     routine
//         .active()
//         .onTrue(getPathCommandSafely(depotIntake).andThen(getPathCommandSafely(depotShoot)));

//     return routine.cmd();
//   }

//   public Command RedDrakeDepotLong() {
//     AutoRoutine routine = autoFactory.newRoutine("CristianoRonaldo.chor");

//     AutoTrajectory depotIntake = depot(routine, Constants.Swerve.Auto.Depot.RedDepotRDrake);
//     AutoTrajectory depotShoot = shoot(routine, Constants.Swerve.Auto.ShootPos.RedDepotToShoot);
//     AutoTrajectory outpostIntake = outpost(routine, Constants.Swerve.Auto.Outpost.RedOutpostL);
//     AutoTrajectory outpostShoot =
//         shoot(routine, Constants.Swerve.Auto.ShootPos.RedOutpostToShootShort);

//     routine
//         .active()
//         .onTrue(
//             getPathCommandSafely(depotIntake)
//                 .andThen(getPathCommandSafely(depotShoot))
//                 .andThen(getPathCommandSafely(outpostIntake))
//                 .andThen(getPathCommandSafely(outpostShoot)));

//     return routine.cmd();
//   }

//   public Command returnBasicShoot() {
//     Command shoot = new ShootBasic(
//         () ->
//             Units.metersToFeet(
//                 Targeting.shootingSpeed(
//                     Constants.Landmarks.RED_HUB,
//                     swerveSubsystem,
//                     Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
//         () ->
//             (Targeting.pointingAtTarget(Constants.Landmarks.RED_HUB, swerveSubsystem)
//                 && lebronShooterSubsystem.isAtSpeed()),
//         lebronShooterSubsystem,
//         intakeSubsystem,
//         hopperSubsystem);

//     return Commands.sequence(shoot.asProxy());
//     // return (new ShootBasic(
//     //     () ->
//     //         Units.metersToFeet(
//     //             Targeting.shootingSpeed(
//     //                 Constants.Landmarks.RED_HUB,
//     //                 swerveSubsystem,
//     //                 Constants.Shooter.TARGETING_CALCULATION_PRECISION)),
//     //     () ->
//     //         (Targeting.pointingAtTarget(Constants.Landmarks.RED_HUB, swerveSubsystem)
//     //             && lebronShooterSubsystem.isAtSpeed()),
//     //     lebronShooterSubsystem,
//     //     intakeSubsystem,
//     //     hopperSubsystem));
//   }

//   // this may not be needed, but is good to have
//   public Command getPathCommandSafely(AutoTrajectory traj) {
//     return traj != null ? traj.cmd() : Commands.none();
//   }

//   // this may not be needed, but is good to have
//   public Command resetPathOdometrySafely(AutoTrajectory traj) {
//     return traj != null ? traj.resetOdometry() : Commands.none();
//   }

//   // if first path is null and odo is not reset, should i add smth to reset next
//   // path or is that OD
//   public void addCommandstoAutoChooser() {
//     // autoChooser.addCmd(
//     // "Red Pedri - Left Side",
//     // () ->
//     // RedPedriLeft(
//     // Constants.Swerve.Auto.Maneuver.RedLeftManeuverL,
//     // Constants.Swerve.Auto.Intake.RedLeftIntakeL,
//     // Constants.Swerve.Auto.ShootPos.RedLeftShoot,
//     // Constants.Swerve.Auto.ClimbPos.RedLeftClimbL));

//     // autoChooser.addCmd(
//     // "Red Fermin - Left Side",
//     // () ->
//     // RedFerminLeft(
//     // Constants.Swerve.Auto.Maneuver.RedLeftManeuverL,
//     // Constants.Swerve.Auto.Intake.RedLeftIntakeMR,
//     // Constants.Swerve.Auto.MiscPaths.RedSweepRight,
//     // Constants.Swerve.Auto.Outpost.RedOutpostM,
//     // Constants.Swerve.Auto.ShootPos.RedOutpostToShoot,
//     // Constants.Swerve.Auto.ClimbPos.RedRightClimbR));

//     // autoChooser.addCmd(
//     // "Red Drake - Right Side",
//     // () ->
//     // RedDrakeRight(
//     // Constants.Swerve.Auto.Outpost.RedOutpostR,
//     // Constants.Swerve.Auto.ShootPos.RedRightShoot,
//     // Constants.Swerve.Auto.ClimbPos.RedRightClimbR));

//     // autoChooser.addCmd("Trial Path", () -> trialPath());

//     // autoChooser.addCmd(
//     //     "Trial Path Two",
//     //     () -> trialPathTwo(Constants.Swerve.Auto.Maneuver.RedRightManeuverR, null, null, null));

//     // autoChooser.addCmd("week0Path", () -> week0Auto());
//     // autoChooser.addCmd("week0PathWithCommands", () -> week0AutoWithCommands());
//     autoChooser.addCmd("doneWeek0Path", () -> doneWeek0Auto());
//     autoChooser.addCmd("doneWeek0PathWithShoot", () -> doneWeek0AutoWithShoot());

//     // autoChooser.addCmd("Red Pedri - depot (right)", () -> RedPedriDepotR());
//     // autoChooser.addCmd("Red Pedri - depot (left)", () -> RedPedriDepotL());
//     // autoChooser.addCmd("Red Pedri - outpost (right)", () -> RedPedriOutpostR());
//     // autoChooser.addCmd("Red Pedri - outpost (left)", () -> RedPedriOutpostL());
//     // autoChooser.addCmd("Red Pedri - short (right)", () -> RedPedriShortR());
//     // autoChooser.addCmd("Red Pedri - short (left)", () -> RedPedriShortL());
//     // autoChooser.addCmd("Red Pedri - mid (right)", () -> RedPedriMidR());
//     // autoChooser.addCmd("Red Pedri - mid (left)", () -> RedPedriMidL());
//     // autoChooser.addCmd("Red Drake - depot (long)", () -> RedDrakeDepotLong());
//     // autoChooser.addCmd("Red Drake - depot (short)", () -> RedDrakeDepotShort());
//     // autoChooser.addCmd("Red Drake - outpost (long)", () -> RedDrakeOutpostLong());
//     // autoChooser.addCmd("Red Drake - outpost (short)", () -> RedDrakeOutpostShort());
//     // autoChooser.addCmd("Nike", () -> Nike());
//   }

//   public AutoChooser getAutoChooser() {
//     return autoChooser;
//   }
// }
