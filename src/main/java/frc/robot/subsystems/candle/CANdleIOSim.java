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
import java.util.Objects;

public class CANdleIOSim implements CANdleIO {
  private AnimationType targetAnimationType = AnimationType.None;
  private AnimationType currentAnimationType = AnimationType.None;

  private RGBWColor targetColor = new RGBWColor();
  private RGBWColor currentColor = new RGBWColor();

  private int startIndex = kFirstLED;
  private int endIndex = kEndLED;

  public CANdleIOSim() {
    // Ensure deterministic startup state (known-off) before any commands call setLEDColor/setLEDAnimation.
    clear();
  } // End CANdleIOLEDs Constructor

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    if (currentAnimationType != targetAnimationType || !Objects.equals(currentColor, targetColor)) {
      currentAnimationType = targetAnimationType;
      currentColor = targetColor;
    }

    inputs.currentAnimationType = currentAnimationType;
    inputs.currentColorRed = currentColor.Red;
    inputs.currentColorGreen = currentColor.Green;
    inputs.currentColorBlue = currentColor.Blue;
    inputs.currentColorWhite = currentColor.White;
    inputs.startLEDIndex = startIndex;
    inputs.endLEDIndex = endIndex;
  } // End updateInputs

  private void setLEDColor(RGBWColor color) {
    // Nothing to set (yet...)
  } // End setLEDColor

  @Override
  public void clear() {
    targetAnimationType = AnimationType.None;
    targetColor = new RGBWColor();

    currentAnimationType = AnimationType.None;
    currentColor = targetColor;

    setLEDColor(new RGBWColor(0, 0, 0, 0));
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
