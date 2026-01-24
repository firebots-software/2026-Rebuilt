package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine simplePathAuto() {
    final AutoRoutine routine = m_factory.newRoutine("CristianoRonaldo.chor");
    final AutoTrajectory simplePath = routine.trajectory("MoveForward.traj");

    routine.active().onTrue(simplePath.resetOdometry().andThen(simplePath.cmd()));
    return routine;
  }

  public Command getPathAsCommand() {
    AutoRoutine routine = simplePathAuto();
    AutoTrajectory trajectory = routine.trajectory("MoveForward.traj");

    return trajectory.cmd();
  }
}
