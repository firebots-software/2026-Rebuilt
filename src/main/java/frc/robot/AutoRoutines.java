package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutines {
  final AutoTrajectory moveForward;
  final AutoRoutine routine;

  public AutoRoutines(AutoFactory factory) {
    routine = factory.newRoutine("CristianoRonaldo.chor");
    moveForward = routine.trajectory("MoveForward.traj");

    routine.active().onTrue(moveForward.resetOdometry().andThen(moveForward.cmd()));
  }

  public AutoRoutine moveForwardAuto() {
    return routine;
  }

  public Command getPathAsCommand() {
    return routine.cmd();
  }
}
