package frc.robot.subsystems.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
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

public class CANdleIOLEDs implements CANdleIO {

  private final CANdle candle;
  
  private AnimationType targetAnimationType;
  private AnimationType currentAnimationType;

  private RGBWColor targetColor;
  private RGBWColor currentColor;

  private int startIndex;
  private int endIndex;

  public CANdleIOLEDs() {
    candle = new CANdle(kDeviceID);

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = kDefaultBrightness;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);
  } // End ExtenderIOSParkMax

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.currentAnimationType = targetAnimationType;
    inputs.targetAnimationType = targetAnimationType;
    inputs.currentColor = currentColor;
    inputs.targetColor = targetColor;
    inputs.startLEDIndex = startIndex;
    inputs.endLEDIndex = endIndex;

    if (currentAnimationType != targetAnimationType || currentColor != targetColor) {
      currentAnimationType = targetAnimationType;
      currentColor = targetColor;

      setLEDAnimation();
    }
  } // End updateInputs

  /** Sets the LED animation on the CANdle using the {@link #currentColor} and the {@link #currentAnimationType}  */
  private void setLEDAnimation() {
    switch (currentAnimationType) {
      default:
      case None:
        setLEDColor(currentColor);
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
  }

  private void setLEDColor(RGBWColor color) {
    candle.setControl(new SolidColor(kFirstLED, kEndLED).withColor(color));
  }

  @Override
  public void setColor(RGBWColor color) {
    targetColor = color;
  }

  @Override
  public void setAnimationType(AnimationType type) {
    targetAnimationType = type;
  }
}
