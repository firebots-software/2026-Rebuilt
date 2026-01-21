package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

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
}
