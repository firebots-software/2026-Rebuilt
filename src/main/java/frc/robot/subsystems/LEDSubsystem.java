package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  // left strip is [8, 23]
  // middle strip is [24, 53] (center of middle is 38)
  // right strip is [51, 76]
  private static final int END_OF_STRIP = 76;

  private CANdle candle = new CANdle(5);
  private LEDState defaultState = LEDState.NONE;
  private LEDState currentState = defaultState;
  private BooleanSupplier active, inRange;

  public enum LEDState {
    NONE,
    ACTIVE,
    ACTIVE_IN_RANGE,
    FLAME,
    RAINBOW
  }

  public LEDSubsystem(BooleanSupplier activeSupplier, BooleanSupplier inRangeSupplier) {
    active = activeSupplier;
    inRange = inRangeSupplier;

    setDefaultCommand(evaluateState());
  }

  public void periodic() {
    DogLog.log("Subsystems/LEDs/Active", active.getAsBoolean());
    DogLog.log("Subsystems/LEDs/InRange", inRange.getAsBoolean());
    DogLog.log("Subsystems/LEDs/CurrentState", currentState.toString());
  }

  public Command evaluateState() {
    if (DriverStation.isDisabled()) return getStateAsCommand(LEDState.FLAME);
    if (DriverStation.isAutonomousEnabled()) return getStateAsCommand(LEDState.RAINBOW);

    if (active.getAsBoolean() && inRange.getAsBoolean())
      return getStateAsCommand(LEDState.ACTIVE_IN_RANGE);
    else if (active.getAsBoolean() && !inRange.getAsBoolean())
      return getStateAsCommand(LEDState.ACTIVE);
    else if (!active.getAsBoolean()) return getStateAsCommand(defaultState);

    return getStateAsCommand(LEDState.NONE);
  }

  public Command getStateAsCommand(LEDState state) {
    return switch (state) {
      case NONE -> runOnce(this::clearAll);
      case ACTIVE -> runOnce(
          () -> {
            clearAll();
            candle.setControl(activeAnimation());
          });
      case ACTIVE_IN_RANGE -> activeInRangeCommand();
      case FLAME -> runOnce(
          () -> {
            clearAll();
            candle.setControl(flame(8, 38, 0).withDirection(AnimationDirectionValue.Forward));
            candle.setControl(
                flame(39, END_OF_STRIP, 1).withDirection(AnimationDirectionValue.Backward));
          });
      case RAINBOW -> runOnce(
          () -> {
            clearAll();
            candle.setControl(new RainbowAnimation(8, END_OF_STRIP));
          });
      default -> runOnce(this::clearAll);
    };
  }

  // helper methods
  public void clearAll() {
    for (int i = 0; i <= 8; i++) candle.setControl(new EmptyAnimation(i));
  }

  private SingleFadeAnimation activeAnimation() {
    return new SingleFadeAnimation(8, END_OF_STRIP).withColor(new RGBWColor(Color.kOrangeRed));
  }

  private SolidColor solidColor(Color color) {
    return new SolidColor(8, END_OF_STRIP).withColor(new RGBWColor(color));
  }

  private Command activeInRangeCommand() {
    return runOnce(this::clearAll)
        .andThen(
            Commands.repeatingSequence(
                runOnce(() -> candle.setControl(solidColor(Color.kOrangeRed))),
                Commands.waitTime(Milliseconds.of(100)),
                runOnce(() -> candle.setControl(solidColor(Color.kWhite))),
                Commands.waitTime(Milliseconds.of(100))));
  }

  private FireAnimation flame(int startIndex, int endIndex, int slot) {
    return new FireAnimation(startIndex, endIndex)
        .withSparking(0.5)
        .withCooling(0.2)
        .withFrameRate(30)
        .withSlot(slot);
  }
}
