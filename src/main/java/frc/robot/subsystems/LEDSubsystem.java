package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  // indices:
  // [0, 7] candle onboard
  // [8, 30] left side strip
  // [31, 59] back strip
  // [60, 76] right side strip
  private static final int END_OF_STRIP = 77;

  private CANdle candle = new CANdle(5);
  private LEDState currentState = LEDState.RAINBOW;
  private BooleanSupplier active, inRange;

  public LEDSubsystem(BooleanSupplier active, BooleanSupplier inRange) {
    this.active = active;
    this.inRange = inRange;
  }

  public void updateState(LEDState newState) {
    currentState = newState;
    candle.setControl(currentState.animation);
  }

  /** sends up to 8 animations to the candle at once (for animations across multiple slots) */
  public void updateState(LEDState... newStates) {
    currentState = newStates[0];
    // only go up to 8 because the candle only has 9 animation slots
    for (int i = 0; i <= 8; i++) candle.setControl(newStates[i].animation);
  }

  /** clear all slots */
  public void clearAll() {
    for (int i = 0; i <= 8; i++) candle.setControl(new EmptyAnimation(i));
  }

  public Command updateStateCommand(LEDState newState) {
    return runOnce(() -> updateState(newState));
  }

  public Command updateStateCommand(LEDState... newStates) {
    return runOnce(() -> updateState(newStates));
  }

  public Command clearAllCommand() {
    return runOnce(this::clearAll);
  }

  public void cycleStrobeColor() {
    // please forgive me
    RGBWColor currentColor = ((StrobeAnimation) (LEDState.ACTIVE_IN_RANGE.animation)).Color;
    ((StrobeAnimation) (LEDState.ACTIVE_IN_RANGE.animation)).Color =
        new RGBWColor(
            currentColor.equals(new RGBWColor(Color.kOrange)) ? Color.kWhite : Color.kOrange);
  }

  @Override
  public void periodic() {
    // if (active.getAsBoolean() && inRange.getAsBoolean()) updateState(LEDState.ACTIVE_IN_RANGE);
    // else if (active.getAsBoolean()) updateState(LEDState.ACTIVE);
    // else updateState(LEDState.NONE);
    updateState(LEDState.ACTIVE_IN_RANGE, LEDState.STROBE_WHITE);

    // switch the color of the active in range animation at a rate of 4hz
    // if (currentState == LEDState.ACTIVE_IN_RANGE && System.currentTimeMillis() % 50 == 0)
    //   cycleStrobeColor();
    DogLog.log("Subsystems/LEDs/CurrentState", currentState.name);
  }

  public enum LEDState {
    /** only works for single-slot animations, use clearAll() for multi-slot */
    NONE("None", new EmptyAnimation(0)),
    ACTIVE(
        "Active",
        new SingleFadeAnimation(8, END_OF_STRIP)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(3)),
    ACTIVE_IN_RANGE(
        "Active (in range)",
        new StrobeAnimation(8, END_OF_STRIP)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(15)),
    STROBE_WHITE(
      "temp",
      new StrobeAnimation(8, END_OF_STRIP)
        .withColor(new RGBWColor(Color.kWhite))
        .withFrameRate(0)
        .withSlot(1)
    ),
    FLAME_LEFT(
        "🔥",
        new FireAnimation(8, 45)
            .withSparking(0.4)
            .withCooling(0.4)
            .withDirection(AnimationDirectionValue.Forward) // backward = outwards from middle
            .withFrameRate(4)
            .withSlot(0)),
    FLAME_RIGHT(
        "🔥",
        new FireAnimation(46, END_OF_STRIP)
            .withSparking(0.4)
            .withCooling(0.4)
            .withDirection(AnimationDirectionValue.Forward) // backward = outwards from middle
            .withFrameRate(4)
            .withSlot(1)),
    SWEEP_LEFT(
        "Sweep",
        new LarsonAnimation(8, 45)
            .withBounceMode(LarsonBounceValue.Center)
            .withColor(new RGBWColor(Color.kGreen))
            .withFrameRate(4)
            .withSlot(0)),
    SWEEP_RIGHT(
        "Sweep",
        new LarsonAnimation(END_OF_STRIP, 46)
            .withBounceMode(LarsonBounceValue.Center)
            .withColor(new RGBWColor(Color.kGreen))
            .withFrameRate(4)
            .withSlot(1)),
    RAINBOW("Rainbow", new RainbowAnimation(8, END_OF_STRIP));

    String name;
    ControlRequest animation;

    LEDState(String name, ControlRequest animation) {
      this.name = name;
      this.animation = animation;
    }
  }
}
