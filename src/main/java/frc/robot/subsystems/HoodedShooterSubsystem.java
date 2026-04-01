package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class HoodedShooterSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final BooleanSupplier redside;
    private final LoggedTalonFX warmup1, warmup2, warmup3, shooter, hood;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private double targetShooterSpeedRPS = 0;
    

    public HoodedShooterSubsystem(CommandSwerveDrivetrain drivetrain, BooleanSupplier redside) {
        this.drivetrain = drivetrain;
        this.redside = redside;
        warmup1 = new LoggedTalonFX("ShooterWarmup1", Constants.Shooter.WARMUP_1_ID, Constants.Swerve.CAN_BUS);
        warmup2 = new LoggedTalonFX("ShooterWarmup2", Constants.Shooter.WARMUP_2_ID, Constants.Swerve.CAN_BUS);
        warmup3 = new LoggedTalonFX("ShooterWarmup3", Constants.Shooter.WARMUP_3_ID, Constants.Swerve.CAN_BUS);
        shooter = warmup3;
        // TODO: fix hood id constant
        hood = new LoggedTalonFX("ShooterHood", Constants.Shooter.HOOD_ID, Constants.Swerve.CAN_BUS);

        Slot0Configs s0c = new Slot0Configs().withKP(Constants.Shooter.KP).withKI(Constants.Shooter.KI).withKD(Constants.Shooter.KD).withKV(Constants.Shooter.KV).withKA(Constants.Shooter.KA);
        CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT).withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast);
        VoltageConfigs voltageConfigs = new VoltageConfigs().withPeakReverseVoltage(0.0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = s0c;
        config.CurrentLimits = clc;
        config.MotorOutput = motorOutputConfigs;
        config.Voltage =voltageConfigs;
    }
    
}
