package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  private static final int STRIP_LENGTH = 69; // nice

  private CANdle candle = new CANdle(0); // TODO
  private LEDState currentState = LEDState.INACTIVE;
  private BooleanSupplier active, inRange;

  public LEDSubsystem(BooleanSupplier active, BooleanSupplier inRange) {
    this.active = active;
    this.inRange = inRange;
  }

  public void updateState(LEDState newState) {
    if (newState == currentState) return;
    candle.setControl(newState.animation);
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
    if (active.getAsBoolean() && inRange.getAsBoolean()) currentState = LEDState.ACTIVE_IN_RANGE;
    else if (active.getAsBoolean()) currentState = LEDState.ACTIVE;
    else currentState = LEDState.INACTIVE;

    // switch the color of the active in range animation at a rate of 4hz
    if (currentState == LEDState.ACTIVE_IN_RANGE && System.currentTimeMillis() % 250 == 0)
      cycleStrobeColor();
    DogLog.log("Subsystems/LEDs/CurrentState", currentState.name);
  }

  public enum LEDState {
    INACTIVE("None", new EmptyAnimation(0)),
    ACTIVE(
        "Active",
        new SingleFadeAnimation(8, STRIP_LENGTH)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(3)),
    ACTIVE_IN_RANGE(
        "Active (in range)",
        new StrobeAnimation(8, STRIP_LENGTH)
            .withColor(new RGBWColor(Color.kOrange))
            .withFrameRate(5));

    String name;
    ControlRequest animation;

    LEDState(String name, ControlRequest animation) {
      this.name = name;
      this.animation = animation;
    }
  }
}
