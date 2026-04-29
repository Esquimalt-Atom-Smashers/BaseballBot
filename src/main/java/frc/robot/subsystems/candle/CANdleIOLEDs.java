package frc.robot.subsystems.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import static frc.robot.subsystems.candle.CANdleConstants.*;
import java.util.Objects;

public class CANdleIOLEDs implements CANdleIO {

  private final CANdle candle;
  
  private AnimationType targetAnimationType = AnimationType.None;
  private AnimationType currentAnimationType = AnimationType.None;

  private RGBWColor targetColor = new RGBWColor();
  private RGBWColor currentColor = new RGBWColor();

  private int startIndex = kFirstLED;
  private int endIndex = kEndLED;

  public CANdleIOLEDs() {
    candle = new CANdle(kDeviceID);

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = kFullBrightness;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);

    // Ensure deterministic startup state (known-off) before any commands call setLEDColor/setLEDAnimation.
    clear();
  } // End CANdleIOLEDs Constructor

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    if (currentAnimationType != targetAnimationType || !Objects.equals(currentColor, targetColor)) {
      currentAnimationType = targetAnimationType;
      currentColor = targetColor;

      setLEDAnimation();
      setLEDColor();
    }

    inputs.currentAnimationType = currentAnimationType;
    inputs.currentColorRed = currentColor.Red;
    inputs.currentColorGreen = currentColor.Green;
    inputs.currentColorBlue = currentColor.Blue;
    inputs.currentColorWhite = currentColor.White;
    inputs.startLEDIndex = startIndex;
    inputs.endLEDIndex = endIndex;
  } // End updateInputs

  /** Sets the LED animation on the CANdle. */
  private void setLEDAnimation() {
    switch (currentAnimationType) {
      default:
      case None:
        candle.setControl(new EmptyAnimation(0));
        break;
      case ColorFlow:
        candle.setControl(
          new ColorFlowAnimation(startIndex, endIndex).withSlot(0)
            .withColor(currentColor)
        );
        break;
      case Rainbow:
        candle.setControl(
          new RainbowAnimation(startIndex, endIndex).withSlot(0)
        );
        break;
      case Strobe:
        candle.setControl(
            new StrobeAnimation(startIndex, endIndex).withSlot(0)
                .withColor(currentColor)
        );
        break;
      case SingleFade:
        candle.setControl(
            new SingleFadeAnimation(startIndex, endIndex).withSlot(0)
                .withColor(currentColor)
        );
        break;
      case RgbFade:
        candle.setControl(
            new RgbFadeAnimation(startIndex, endIndex).withSlot(0)
        );
        break;
      case Twinkle:
        candle.setControl(
          new TwinkleAnimation(startIndex, endIndex).withSlot(0)
            .withColor(currentColor)
        );
        break;
      case TwinkleOff:
        candle.setControl(
          new TwinkleOffAnimation(startIndex, endIndex).withSlot(0)
            .withColor(currentColor)
        );
        break;
      case Larson:
        candle.setControl(
          new LarsonAnimation(startIndex, endIndex).withSlot(0)
            .withColor(currentColor)
        );
        break;
      case Fire:
        candle.setControl(
          new FireAnimation(startIndex, endIndex).withSlot(0)
        );
        break;
    }
  } // End setLEDAnimation

  private void setLEDColor() {
    candle.setControl(new SolidColor(startIndex, endIndex).withColor(currentColor));
  } // End setLEDColor

  @Override
  public void clear() {
    targetAnimationType = AnimationType.None;
    targetColor = new RGBWColor();

    currentAnimationType = AnimationType.None;
    currentColor = targetColor;
  } // End clear

  @Override
  public void setColor(RGBWColor color) {
    targetColor = color != null ? color : new RGBWColor();
  } // End setColor

  @Override
  public void setAnimationType(AnimationType type) {
    targetAnimationType = type != null ? type : AnimationType.None;
  } // End setAnimationType
}
