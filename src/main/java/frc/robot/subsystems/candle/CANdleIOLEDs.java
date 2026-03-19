package frc.robot.subsystems.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import static frc.robot.subsystems.candle.CANdleConstants.*;

public class CANdleIOLEDs implements CANdleIO {

  private final CANdle candle;
  
  private AnimationType targetAnimationType;
  private AnimationType currentAnimationType;

  public CANdleIOLEDs() {
    candle = new CANdle(CANDLE_LED_DEVICE_ID);

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = LED_DEFAULT_BRIGHTNESS;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(config);
  } // End ExtenderIOSParkMax

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.currentAnimationType = targetAnimationType;
    inputs.targetAnimationType = targetAnimationType;
  } // End updateInputs

  @Override
  public void setColor(RGBWColor colour) {
    candle.setControl(new SolidColor(FIRST_LED_INDEX, LAST_LED_INDEX).withColor(colour));
  }

  @Override
  public void setAnimationType(AnimationType type) {
    targetAnimationType = type;
  }
}
