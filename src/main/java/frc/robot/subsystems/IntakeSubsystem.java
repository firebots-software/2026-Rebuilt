package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    private LoggedTalonFX leftMotor, rightMotor, master;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);;

    public IntakeSubsystem() {
        leftMotor = new LoggedTalonFX("leftMotor", 35, "rio");
        rightMotor = new LoggedTalonFX("rightMotor", 34, "rio");

        Slot0Configs s0c = new Slot0Configs();

        Follower follower = new Follower(35, MotorAlignmentValue.Opposed);
        master = leftMotor;
        rightMotor.setControl(follower);

        MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        MotionMagicConfigs mmc = new MotionMagicConfigs();

        master.updateCurrentLimits(30, 30);
        rightMotor.updateCurrentLimits(30, 30);

        TalonFXConfigurator m1Config = master.getConfigurator();
        TalonFXConfigurator m2Config = rightMotor.getConfigurator();

        m1Config.apply(s0c);
        m1Config.apply(moc);
        m1Config.apply(mmc);
        m2Config.apply(s0c);
        m2Config.apply(moc);
        m2Config.apply(mmc);
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null)
            instance = new IntakeSubsystem();
        return instance;
    }

    public void setSpeed(double speed) {
        master.setControl(velocityRequest.withVelocity(speed));
        DogLog.log("intakeSpeed", speed);
    }

    public void stop() {
        master.setControl(velocityRequest.withVelocity(0));
        DogLog.log("intakeSpeed", 0);
    }
}
