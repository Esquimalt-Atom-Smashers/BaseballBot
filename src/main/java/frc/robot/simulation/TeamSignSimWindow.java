package frc.robot.simulation;

import java.awt.Color;
import java.awt.Font;
import java.util.function.Supplier;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.WindowConstants;

/**
 * Desktop-only pop-out that mimics the driver-station LED team sign. Open one per
 * {@link Role} from sim (for example {@code Robot#simulationInit}).
 */
public final class TeamSignSimWindow {

	/** Which RP fuel numerator the window shows (blue hub vs red hub score). */
	public enum Role {
		BLUE_RP_FIELD("Team sign (sim) — Blue RP"),
		RED_RP_FIELD("Team sign (sim) — Red RP");

		private final String defaultTitle;

		Role(String defaultTitle) {
			this.defaultTitle = defaultTitle;
		}

		String defaultTitle() {
			return defaultTitle;
		}
	}

	private static final int kDefaultWindowWidthPx = 330;
	private static final Color kLedRed = new Color(255, 40, 40);

	private static final class Slot {
		JFrame frame;
		Timer refreshTimer;
		JLabel label;
	}

	private static final Slot[] slots = new Slot[Role.values().length];

	static {
		for (int i = 0; i < slots.length; i++) {
			slots[i] = new Slot();
		}
	}

	private TeamSignSimWindow() {}

	/**
	 * Opens or focuses the window for {@code role} and refreshes from {@code lineSupplier} (~10 Hz).
	 */
	public static void open(Role role, Supplier<String> lineSupplier) {
		open(role, role.defaultTitle(), kLedRed, lineSupplier);
	}

	/**
	 * Opens or focuses the window for {@code role} with a custom title and LED foreground color.
	 */
	public static void open(Role role, String title, Color foreground, Supplier<String> lineSupplier) {
		int ordinal = role.ordinal();
		SwingUtilities.invokeLater(
				() -> {
					Slot slot = slots[ordinal];
					if (slot.frame == null) {
						slot.frame = new JFrame(title);
						slot.frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
						slot.label = new JLabel(" ", SwingConstants.CENTER);
						slot.label.setOpaque(true);
						slot.label.setBackground(Color.BLACK);
						slot.label.setForeground(foreground);
						slot.label.setFont(new Font(Font.MONOSPACED, Font.BOLD, 28));
						slot.frame.add(slot.label);
						slot.frame.pack();
						slot.frame.setSize(Math.max(kDefaultWindowWidthPx, slot.frame.getWidth()), slot.frame.getHeight());
						slot.frame.setMinimumSize(slot.frame.getSize());
						int y = 48 + ordinal * (slot.frame.getHeight() + 16);
						slot.frame.setLocation(80, y);
						slot.refreshTimer =
								new Timer(
										100,
										e -> {
											String text = lineSupplier.get();
											String next = text == null ? " " : text;
											if (!next.equals(slot.label.getText())) {
												slot.label.setText(next);
											}
										});
						slot.refreshTimer.start();
					} else {
						slot.frame.setTitle(title);
						slot.label.setForeground(foreground);
					}
					slot.frame.setVisible(true);
					slot.frame.toFront();
				});
	} // End open

	/** Stops refresh and disposes all opened windows. */
	public static void close() {
		SwingUtilities.invokeLater(
				() -> {
					for (Slot slot : slots) {
						if (slot.refreshTimer != null) {
							slot.refreshTimer.stop();
							slot.refreshTimer = null;
						}
						if (slot.frame != null) {
							slot.frame.dispose();
							slot.frame = null;
						}
						slot.label = null;
					}
				});
	} // End close
}
