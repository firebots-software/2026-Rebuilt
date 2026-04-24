package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  // left strip is [8, 23]
  // middle strip is [24, 53] (center of middle is 38)
  // right strip is [51, 76]
  private static final int END_OF_STRIP = 76;

  private CANdle candle = new CANdle(5);
  private LEDState currentState = LEDState.NONE;
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
  }

  public void periodic() {
    LEDState computedState = computeState();
    if (computedState != currentState) {
      currentState = computedState;
      applyState(computedState);
    }
    if (currentState == LEDState.ACTIVE_IN_RANGE) activeInRangeAnimation();

    DogLog.log("Subsystems/LEDs/Active", active.getAsBoolean());
    DogLog.log("Subsystems/LEDs/InRange", inRange.getAsBoolean());
    DogLog.log("Subsystems/LEDs/CurrentState", currentState.toString());
  }

  public LEDState computeState() {
    if (DriverStation.isDisabled()) return LEDState.FLAME;
    if (DriverStation.isAutonomousEnabled()) return LEDState.RAINBOW;

    if (active.getAsBoolean() && inRange.getAsBoolean()) return LEDState.ACTIVE_IN_RANGE;
    else if (active.getAsBoolean() && !inRange.getAsBoolean()) return LEDState.ACTIVE;

    return LEDState.NONE;
  }

  public void applyState(LEDState state) {
    clearAll();
    // active in range animation handled in periodic
    switch (state) {
      case ACTIVE -> candle.setControl(activeAnimation());
      case FLAME -> {
        candle.setControl(flame(8, 38, 0, false));
        candle.setControl(flame(39, END_OF_STRIP, 1, true));
      }
      case RAINBOW -> candle.setControl(new RainbowAnimation(8, END_OF_STRIP));
      default -> clearAll();
    }
  }

  // helper methods
  public void clearAll() {
    for (int i = 0; i <= 7; i++) candle.setControl(new EmptyAnimation(i));
  }

  private SingleFadeAnimation activeAnimation() {
    return new SingleFadeAnimation(8, END_OF_STRIP).withColor(new RGBWColor(Color.kRed));
  }

  private SolidColor solidColor(Color color) {
    return new SolidColor(8, END_OF_STRIP).withColor(new RGBWColor(color));
  }

  private void activeInRangeAnimation() {
    boolean orange = ((int) (Timer.getFPGATimestamp() * 10)) % 2 == 0;
    candle.setControl(solidColor(orange ? Color.kRed : Color.kWhite));
  }

  private FireAnimation flame(int startIndex, int endIndex, int slot, boolean backward) {
    return new FireAnimation(startIndex, endIndex)
        .withSparking(0.5)
        .withCooling(0.2)
        .withFrameRate(30)
        .withDirection(
            backward ? AnimationDirectionValue.Backward : AnimationDirectionValue.Forward)
        .withSlot(slot);
  }
}
