package frc.robot.simulation;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Package-private factory and adapters for {@link HumanControlGamepad}. */
final class HumanControlGamepadImpl {

	private HumanControlGamepadImpl() {}

	static HumanControlGamepad forPort(int port, boolean useDirectInput) {
		return useDirectInput ? new DInputXboxAdapter(new CommandJoystick(port)) : new XboxAdapter(new CommandXboxController(port));
	} // End forPort

	private record XboxAdapter(CommandXboxController xbox) implements HumanControlGamepad {
		@Override
		public double getLeftX() {
			return xbox.getLeftX();
		}

		@Override
		public double getLeftY() {
			return xbox.getLeftY();
		}

		@Override
		public double getRightX() {
			return xbox.getRightX();
		}

		@Override
		public double getRightTriggerAxis() {
			return xbox.getRightTriggerAxis();
		}

		@Override
		public boolean getAButton() {
			return xbox.getHID().getAButton();
		}
	} // End XboxAdapter

	/**
	 * Windows DirectInput / HID for Xbox-style pads: left stick X/Y; rotation often on Z (raw axis 2). Right trigger
	 * is a digital button ({@code BUTTON_RIGHT_TURBO}) for full turbo when held; set indices from the Driver Station USB tab.
	 */
	private static final class DInputXboxAdapter implements HumanControlGamepad {
		private static final int AXIS_LEFT_X = 0;
		private static final int AXIS_LEFT_Y = 1;
		/** Right-stick horizontal / omega input (often labeled Z Axis in joy.cpl, not Rx). */
		private static final int AXIS_ROTATION_Z = 2;
		/** WPILib raw button index for right trigger as a digital control (no analog RT on this HID layout). */
		private static final int BUTTON_RIGHT_TURBO = 8;
		private static final int BUTTON_A = 2;

		private final CommandJoystick joy;

		DInputXboxAdapter(CommandJoystick joy) {
			this.joy = joy;
		}

		@Override
		public double getLeftX() {
			return joy.getHID().getRawAxis(AXIS_LEFT_X);
		}

		@Override
		public double getLeftY() {
			// Match XInput stick polarity after {@code runHumanControlSim} negates for field frame (was inverted vs X).
			return joy.getHID().getRawAxis(AXIS_LEFT_Y);
		}

		@Override
		public double getRightX() {
			return joy.getHID().getRawAxis(AXIS_ROTATION_Z);
		}

		@Override
		public double getRightTriggerAxis() {
			// DirectInput: RT is a button → full turbo (max speed cap) when held, base turbo when released.
			return joy.getHID().getRawButton(BUTTON_RIGHT_TURBO) ? 1.0 : 0.0;
		}

		@Override
		public boolean getAButton() {
			return joy.getHID().getRawButton(BUTTON_A);
		}
	} // End DInputXboxAdapter
} // End HumanControlGamepadImpl
