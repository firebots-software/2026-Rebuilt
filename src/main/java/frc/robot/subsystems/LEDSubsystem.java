package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  /**
   * indices: [0, 7] candle onboard [8, 30] left side strip [31, 59] back strip [60, 76] right side
   * strip
   */
  private static final int END_OF_STRIP = 76;

  private CANdle candle = new CANdle(0); // TODO
  private LEDState currentState = LEDState.INACTIVE;
  private BooleanSupplier active, inRange;

  public LEDSubsystem(BooleanSupplier active, BooleanSupplier inRange) {
    this.active = active;
    this.inRange = inRange;
  }

  public void updateState(LEDState newState) {
    if (newState == currentState) return;
    currentState = newState;
    candle.setControl(newState.animation);
  }

  /** sends up to 8 animations to the candle at once (for animations across multiple slots) */
  public void updateState(LEDState... newStates) {
    currentState = newStates[0];
    // only go up to 8 because the candle only has 9 animation slots
    for (int i = 0; i <= 8; i++) candle.setControl(newStates[i].animation);
  }

  public Command updateStateCommand(LEDState newState) {
    return runOnce(() -> updateState(newState));
  }

  public Command updateStateCommand(LEDState... newStates) {
    return runOnce(() -> updateState(newStates));
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
    if (active.getAsBoolean() && inRange.getAsBoolean()) updateState(LEDState.ACTIVE_IN_RANGE);
    else if (active.getAsBoolean()) updateState(LEDState.ACTIVE);
    else updateState(LEDState.INACTIVE);

    // switch the color of the active in range animation at a rate of 4hz
    if (currentState == LEDState.ACTIVE_IN_RANGE && System.currentTimeMillis() % 250 == 0)
      cycleStrobeColor();
    DogLog.log("Subsystems/LEDs/CurrentState", currentState.name);
  }

  public enum LEDState {
    INACTIVE("None", new EmptyAnimation(0)),
    ACTIVE(
        "Active",
        new SingleFadeAnimation(8, END_OF_STRIP)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(3)),
    ACTIVE_IN_RANGE(
        "Active (in range)",
        new StrobeAnimation(8, END_OF_STRIP)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(5)),
    FLAME_LEFT(
        "🔥",
        new FireAnimation(8, 45)
            .withSparking(0.4)
            .withCooling(0.4)
            .withFrameRate(4)
            .withDirection(AnimationDirectionValue.Forward) // backward = outwards from middle
        ),
    FLAME_RIGHT(
        "🔥",
        new FireAnimation(46, END_OF_STRIP)
            .withSparking(0.4)
            .withCooling(0.4)
            .withFrameRate(4)
            .withDirection(AnimationDirectionValue.Forward) // backward = outwards from middle
        );

    String name;
    ControlRequest animation;

    LEDState(String name, ControlRequest animation) {
      this.name = name;
      this.animation = animation;
    }
  }
}
