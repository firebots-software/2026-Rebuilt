package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {
  private static final int STRIP_LENGTH = 41;

  private static FireAnimation FLAME =
      new FireAnimation(8, STRIP_LENGTH).withFrameRate(15).withSparking(0.4).withCooling(0.4);

  private static SingleFadeAnimation MATCH_IDLE =
      new SingleFadeAnimation(8, STRIP_LENGTH).withColor(new RGBWColor(255, 255, 255));
  private static StrobeAnimation BLINK = new StrobeAnimation(8, STRIP_LENGTH);
  private static SolidColor SOLID = new SolidColor(8, STRIP_LENGTH);

  private CANdle candle;
  private LEDState currentState;
  private LEDStateGroup currentStateGroup;

  public LEDSubsystem() {
    candle = new CANdle(39);

    // reset all leds on init
    LEDStateGroup.LEDS_OFF.run(candle);
  }

  @Override
  public void periodic() {
    if (currentState != null) currentState.run(candle);
    if (currentStateGroup != null) currentStateGroup.run(candle);
    DogLog.log("Subsystems/LEDs/runningAnimation", currentState != null ? currentState.name : null);
    DogLog.log(
        "Subsystems/LEDs/runningAnimationGroup",
        currentStateGroup != null ? currentStateGroup.name : null);
  }

  public void setState(LEDState state) {
    currentState = state;
    currentStateGroup = null;
  }

  public void setState(LEDStateGroup stateGroup) {
    currentStateGroup = stateGroup;
    currentState = null;
  }

  public void updateAlliance(Optional<Alliance> alliance) {
    if (alliance.isEmpty()) return;
    if (alliance.get() == Alliance.Red) {
      RGBWColor red = new RGBWColor(255, 0, 0);
      MATCH_IDLE.Color = red;
      BLINK.Color = red;
      SOLID.Color = red;
    } else if (alliance.get() == Alliance.Blue) {
      RGBWColor blue = new RGBWColor(0, 0, 255);
      MATCH_IDLE.Color = blue;
      BLINK.Color = blue;
      SOLID.Color = blue;
    }
  }

  /** Enum for animation info */
  public enum LEDState {
    NONE_SLOT_0("LEDs off (slot 0)", new EmptyAnimation(0)),
    NONE_SLOT_1("LEDs off (slot 1)", new EmptyAnimation(1)),
    NONE_SLOT_2("LEDs off (slot 2)", new EmptyAnimation(2)),
    NONE_SLOT_3("LEDs off (slot 3)", new EmptyAnimation(3)),

    IDLE("Idle", MATCH_IDLE),
    ALLIANCE_BLINK("Blinking", BLINK),
    ALLIANCE_BLINK_RAPID("Blinking", BLINK.clone().withFrameRate(15)),
    ALLIANCE_SOLID("Solid", SOLID),

    FLAME_LEFT(
        "Flame (left side LEDs)",
        FLAME
            .clone()
            .withLEDStartIndex(28)
            .withLEDEndIndex(STRIP_LENGTH)
            .withFrameRate(15)
            .withDirection(AnimationDirectionValue.Backward)
            .withSlot(0)),
    FLAME_RIGHT(
        "Flame (right side LEDs)",
        FLAME.clone().withLEDStartIndex(8).withLEDEndIndex(27).withFrameRate(15).withSlot(2));

    String name;
    ControlRequest animation;

    LEDState(String name, ControlRequest animation) {
      this.name = name;
      this.animation = animation;
    }

    void run(CANdle candle) {
      candle.setControl(animation);
    }
  }

  /**
   * Represents a group of LEDStates to run multiple at the same time, since some animations need to
   * run across multiple slots simultaneously
   */
  public enum LEDStateGroup {
    LEDS_OFF(
        "LEDs off",
        LEDState.NONE_SLOT_0,
        LEDState.NONE_SLOT_1,
        LEDState.NONE_SLOT_2,
        LEDState.NONE_SLOT_3),
    FLAME(
        "Flame",
        LEDState.FLAME_LEFT,
        /* LEDState.FLAME_MIDDLE_LEFT, LEDState.FLAME_MIDDLE_IGHT, */ LEDState.FLAME_RIGHT);

    String name;
    LEDState[] animations;

    LEDStateGroup(String name, LEDState... animations) {
      this.name = name;
      this.animations = animations;
    }

    void run(CANdle candle) {
      for (LEDState ledState : animations) ledState.run(candle);
    }
  }
}
