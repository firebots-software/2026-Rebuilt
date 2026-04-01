package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class HoodedShooterSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final BooleanSupplier redside;
    private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private double targetShooterSpeedRPS = 0;
    

    public  HoodedShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
        this.drivetrain = drivetrain;
        this.redside = redside;
        warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.WARMUP_1_ID, Constants.Swerve.CAN_BUS);
        warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.WARMUP_2_ID, Constants.Swerve.CAN_BUS);
        warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.WARMUP_3_ID, Constants.Swerve.CAN_BUS);
        shooter = warmup3;
        // TODO: fix hood id constant
        hood = new LoggedTalonFX("ShooterHood", Constants.Shooter.HOOD_ID, Constants.Swerve.CAN_BUS);

    }
    
}
