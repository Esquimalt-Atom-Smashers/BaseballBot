package frc.robot.subsystems.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

    setColor(new RGBWColor(Color.kWhite));
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

      switch (currentAnimationType) {
        default:
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
        case Fire:
          candle.setControl(
            new FireAnimation(startIndex, endIndex).withSlot(0)
          );
          break;
      }
    }
  } // End updateInputs

  @Override
  public void setColor(RGBWColor colour) {
    candle.setControl(new SolidColor(kFirstLED, kEndLED).withColor(colour));
  }

  @Override
  public void setAnimationType(AnimationType type) {
    targetAnimationType = type;
  }
}
