package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.LoggedTalonFX;

import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Intake extends SubsystemBase {
    private LoggedTalonFX forwardMotor;
    private LoggedTalonFX oppositeMotor;

    public LoggedTalonFX master;

    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    private final VoltageOut m_voltage = new VoltageOut(0);

  public Intake() {
    forwardMotor = new LoggedTalonFX("subsystems/Intake/forwardMotor", IntakeConstants.kForwardMotorPort); //add constants
    oppositeMotor = new LoggedTalonFX("subsystems/Intake/oppositeMotor", IntakeConstants.kOppositeMotorPort); //add constants

    Follower follower = new Follower(Constants.IntakeConstants.kForwardMotorPort, MotorAlignmentValue.Aligned);
    oppositeMotor.setControl(follower);

    master = forwardMotor;
    
    forwardMotor.updateCurrentLimits(IntakeConstants.STATOR_CURRENT_LIMIT, IntakeConstants.SUPPLY_CURRENT_LIMIT); //constants
    oppositeMotor.updateCurrentLimits(IntakeConstants.STATOR_CURRENT_LIMIT,IntakeConstants.SUPPLY_CURRENT_LIMIT);
  }

  public void runDutyCycle(double dutyCycle) {
    forwardMotor.setControl(m_dutyCycle.withOutput(dutyCycle));
  }

  public void runVoltage(double voltage) {
    forwardMotor.setControl(m_voltage.withOutput(voltage));
  }

  @Override
  public void periodic() {
  }
}
