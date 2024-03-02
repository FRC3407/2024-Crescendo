package frc.robot.controls;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.Input.InputDevice;
import frc.robot.controls.Input.InputMap;
import frc.utils.SenderNT;

/**
 * ControlSchemeManager manages an array of control schemes and automatically
 * assigns input-action bindings depending on the inputs that are available
 */
public class ControlSchemeManager implements Sendable {

	/**
	 * ControlSchemeBase defines the requirements for a compatible control scheme
	 */
	public static interface ControlSchemeBase {
		public static interface Compat_F {
			public InputDevice[] test(InputDevice... inputs);
		}

		public static interface Setup_F {
			public void run(InputDevice... inputs);
		}

		/**
		 * Determines if the inputs currently connected to the DS are sufficient to
		 * initialize the control scheme
		 * 
		 * @param inputs an array of available inputs - usually of length
		 *               DriverStation.kJoystickPorts
		 * @return an array of inputs that should be passed to setup() to initialize the
		 *         control scheme, or null if the requirements are not met
		 */
		public InputDevice[] compatible(InputDevice... inputs);

		/**
		 * Initializes the control scheme.
		 * 
		 * @param inputs The input array as returned directly from compatible(). The
		 *               order of inputs in that array is the same as in this array
		 */
		public void setup(InputDevice... inputs);

		/** Optional callback for deiniting the scheduled bindings */
		default public void shutdown() {
		}

		/**
		 * What name should the selector use to identify this control scheme
		 * 
		 * @return The name or description of the control scheme
		 */
		public String getDesc();
	}

	public static class ControlScheme implements ControlSchemeBase {
		private final Compat_F compatibility;
		private final Setup_F setup;
		private final Runnable shutdown;
		private final String desc;

		/**
		 * @param d Description of the control scheme.
		 * @param c Compatibility function.
		 * @param s Setup function.
		 */
		public ControlScheme(String d, Compat_F c, Setup_F s) {
			this(d, c, s, null);
		}

		/**
		 * @param d Description of the control scheme.
		 * @param c Compatibility function.
		 * @param s Setup function.
		 * @param e Shutdown function.
		 */
		public ControlScheme(String d, Compat_F c, Setup_F s, Runnable e) {
			this.compatibility = c;
			this.setup = s;
			this.shutdown = e;
			this.desc = d;
		}

		@Override
		public String getDesc() {
			return this.desc;
		}

		@Override
		public InputDevice[] compatible(InputDevice... inputs) {
			return this.compatibility.test(inputs);
		}

		@Override
		public void setup(InputDevice... inputs) {
			this.setup.run(inputs);
		}

		@Override
		public void shutdown() {
			if (this.shutdown != null) {
				this.shutdown.run();
			}
		}

	}

	/**
	 * the AutomatedTester class is designed for automated testing of control
	 * schemes. It takes input requirements and matches them with available input
	 * devices. If all requirements are met, it returns an array of matched
	 * InputDevice objects; otherwise, it returns null.
	 */
	public static class AutomatedTester implements ControlSchemeBase.Compat_F {
		private final InputMap[] requirements;
		private final InputDevice[] buff; // The array [reference] is final, but the assignments are not.

		/**
		 * Constructor for AutomatedTester.
		 *
		 * @param reqs InputMap requirements.
		 */
		public AutomatedTester(InputMap... reqs) {
			this.requirements = reqs;
			this.buff = new InputDevice[reqs.length];
		}

		/**
		 * This method is essentially a big matching function.
		 *
		 * @param inputs Input devices to test.
		 * @return An array of matched InputDevices or null if not all requirements are
		 *         met.
		 */
		@Override
		public InputDevice[] test(InputDevice... inputs) {
			int[] avail, reqs = new int[this.requirements.length]; // Look-up tables for inputs (param) and
																	// this.requirements, respectively
			int found = 0;

			if (inputs.length == DriverStation.kJoystickPorts) {
				avail = new int[] { 0, 1, 2, 3, 4, 5 }; // Available ports: 0 through (DriverStation.kJoystickPorts - 1)
			} else {
				avail = new int[inputs.length];
				for (int i = 0; i < avail.length; i++) {
					avail[i] = inputs[i].getPort();
				}
			}

			for (int i = 0; i < reqs.length; i++) {
				reqs[i] = i;
			}

			for (int i = found; i < avail.length; i++) { // For each remaining port:
				int p = avail[i];
				for (int r = found; r < reqs.length; r++) { // For each remaining requirement:
					if (this.requirements[reqs[r]].compatible(p)) { // If compatible:
						this.buff[reqs[r]] = inputs[p];
						if (i != found) {
							for (int q = i; q > found; q--) { // Start at i, work backwards until @ found
								avail[q] = avail[q - 1]; // Bump all values before i up an index
							}
						}
						if (r != found) {
							for (int q = r; q > found; q--) { // Same as above but for the reqs look-up table
								reqs[q] = reqs[q - 1];
							}
						}
						found++;
						break;
					}
				}
				if (found == this.requirements.length) {
					return this.buff;
				}
			}
			return null;
		}
	}

	/**
	 * Enum representing different preferences for handling ambiguous control scheme
	 * solutions.
	 */
	public static enum AmbiguousSolution {
		NONE,
		PREFER_COMPLEX,
		PREFER_SIMPLE
	}

	private static final Thread DUMMY_THREAD = new Thread();

	private final ArrayList<ControlSchemeBase> schemes = new ArrayList<>();
	private final InputDevice[] inputs = new InputDevice[DriverStation.kJoystickPorts]; // make static?
	private SendableChooser<Integer> options = new SendableChooser<>();
	private Thread searcher;
	private String applied = "None";
	private AmbiguousSolution ambg_preference = AmbiguousSolution.NONE;

	/**
	 * Initializes options and input devices.
	 */
	public ControlSchemeManager() {
		this.options.setDefaultOption("Automatic", Integer.valueOf(-1));
		for (int i = 0; i < this.inputs.length; i++) {
			this.inputs[i] = new InputDevice(i);
		}
	}

	@Override
	public void initSendable(SendableBuilder b) {
		b.addStringProperty("Applied Control Scheme", () -> this.applied, null);
	}

	/**
	 * Add a new control scheme to the manager.
	 * 
	 * @param c ControlSchemeBase to be added.
	 */
	public void addScheme(ControlSchemeBase c) {
		this.options.addOption(c.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(c);
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility and setup
	 * functions.
	 * 
	 * @param d Description of the control scheme.
	 * @param c Compatibility function.
	 * @param s Setup function.
	 */
	public void addScheme(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s) {
		this.options.addOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s));
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param d Description of the control scheme.
	 * @param c Compatibility function.
	 * @param s Setup function.
	 * @param e Shutdown function.
	 */
	public void addScheme(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s, Runnable e) {
		this.options.addOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s, e));
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param d    Description of the control scheme.
	 * @param s    Setup function.
	 * @param reqs InputMap requirements.
	 */
	public void addScheme(String d, ControlSchemeBase.Setup_F s, InputMap... reqs) {
		this.addScheme(d, new AutomatedTester(reqs), s);
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param d    Description of the control scheme.
	 * @param s    Setup function.
	 * @param e    Shutdown function.
	 * @param reqs InputMap requirements.
	 */
	public void addScheme(String d, ControlSchemeBase.Setup_F s, Runnable e, InputMap... reqs) {
		this.addScheme(d, new AutomatedTester(reqs), s, e);
	}

	/**
	 * Set the default control scheme.
	 * 
	 * @param c ControlSchemeBase to be set as default.
	 */
	public void setDefault(ControlSchemeBase c) {
		this.options.setDefaultOption(c.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(c);
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new scheme.
	 * 
	 * @param d Description of the control scheme.
	 * @param c Compatibility function.
	 * @param s Setup function.
	 */
	public void setDefault(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s) {
		this.options.setDefaultOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s));
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new scheme.
	 * 
	 * @param d Description of the control scheme.
	 * @param c Compatibility function.
	 * @param s Setup function.
	 * @param e Shutdown function.
	 */
	public void setDefault(String d, ControlSchemeBase.Compat_F c, ControlSchemeBase.Setup_F s, Runnable e) {
		this.options.setDefaultOption(d, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(d, c, s, e));
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new
	 * AutomatedTester.
	 * 
	 * @param d    Description of the control scheme.
	 * @param s    Setup function.
	 * @param reqs InputMap requirements.
	 */
	public void setDefault(String d, ControlSchemeBase.Setup_F s, InputMap... reqs) {
		this.setDefault(d, new AutomatedTester(reqs), s);
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new
	 * AutomatedTester.
	 * 
	 * @param d    Description of the control scheme.
	 * @param s    Setup function.
	 * @param e    Shutdown function.
	 * @param reqs InputMap requirements.
	 */
	public void setDefault(String d, ControlSchemeBase.Setup_F s, Runnable e, InputMap... reqs) {
		this.setDefault(d, new AutomatedTester(reqs), s, e);
	}

	/**
	 * Publish the control scheme selector to SmartDashboard.
	 */
	public void publishSelector() {
		this.publishSelector("Control Scheme");
	}

	/**
	 * Overloaded method to publish the control scheme selector to SmartDashboard.
	 * 
	 * @param n Name of the control scheme selector.
	 */
	public void publishSelector(String n) {
		SmartDashboard.putData(n + "/Selector", this.options);
		SmartDashboard.putData(n, this);
	}

	/**
	 * Overloaded method to publish the control scheme selector to SmartDashboard.
	 * 
	 * @param nt Specific Network table to publish to.
	 */
	public void publishSelector(SenderNT nt) {
		this.publishSelector(nt, "Control Scheme");
	}

	/**
	 * Overloaded method to publish the control scheme selector to SmartDashboard.
	 * 
	 * @param nt Specific Network table to publish to.
	 * @param n  Name of the control scheme selector.
	 */
	public void publishSelector(SenderNT nt, String n) {
		nt.putData(n + "/Selector", this.options);
		nt.putData(n, this);
	}

	/**
	 * Set the preference for handling ambiguous control scheme solutions.
	 * 
	 * @param s AmbiguousSolution preference.
	 */
	public void setAmbiguousSolution(AmbiguousSolution s) {
		this.ambg_preference = s;
	}

	/**
	 * Clear the selection by resetting options and clearing schemes.
	 */
	synchronized public void clearSelection() {
		this.options = new SendableChooser<>();
		this.options.setDefaultOption("Automatic", Integer.valueOf(-1));
		this.schemes.clear();
	}

	/**
	 * Schedules a new Continuous Worker to select the most compatible control
	 * scheme
	 */
	public synchronized void loopScheme() {
		ContinuousSelectionBuffer buff = new ContinuousSelectionBuffer();
		scheduleContinuousWorker(buff);
	}

	/**
	 * Class representing a buffer for storing selected control schemes.
	 */
	private class SelectionBuffer {
		public InputDevice[] devices = null, buff = null;
	}

	/**
	 * Class representing a continuous selection buffer with additional tracking
	 * information.
	 */
	private class ContinuousSelectionBuffer extends SelectionBuffer {
		int prev_selected = -1, prev_active_id = -1;
		boolean has_any = false;
	}

	private static int lastSelectedCompat;
	private static int lastSelectedId;

	/**
	 * This method schedules the continuous worker responsible for selecting and
	 * configuring control schemes. It checks compatibility, prioritizes schemes
	 * based on complexity or simplicity, and handles ambiguous cases. The selected
	 * scheme is set up for use.
	 * 
	 * @param sel ContinuousSelectionBuffer for additional tracking information.
	 */
	private void scheduleContinuousWorker(ContinuousSelectionBuffer sel) {
		int id = this.options.getSelected();

		// Check if there are no previous selections or if the current selection has
		// changed
		if (!sel.has_any || (sel.prev_selected != id && sel.prev_active_id != id)) {
			if (id < 0) {
				sel.devices = sel.buff = null;
				int compat = -1;

				// Iterate through available schemes
				for (int i = 0; i < this.schemes.size(); i++) {
					sel.buff = this.schemes.get(i).compatible(this.inputs);

					// If a compatible scheme is found
					if (sel.buff != null && sel.buff.length > 0) {
						switch (this.ambg_preference) {
							case PREFER_COMPLEX:
								if (sel.devices == null || sel.buff.length > sel.devices.length) {
									sel.devices = sel.buff;
									compat = i;
								}
								break;
							case PREFER_SIMPLE:
								if (sel.devices == null || sel.buff.length < sel.devices.length) {
									sel.devices = sel.buff;
									compat = i;
								}
								break;
							default:
							case NONE:
								compat = (compat == -1) ? i : -2;
								sel.devices = sel.buff;
						}
					}
				}

				// If a compatible scheme was found
				if (compat >= 0) {
					if (sel.has_any) {
						// Shut down the previously active scheme
						this.schemes.get(sel.prev_active_id).shutdown();
					}
					// Set up the newly compatible scheme
					if (compat != lastSelectedCompat) {
					this.schemes.get(compat).setup(sel.devices);
					this.applied = this.schemes.get(compat).getDesc();
					sel.prev_active_id = compat;
					sel.prev_selected = id;
					sel.has_any = true;
					}
					lastSelectedCompat = compat;
				} else if (compat < -1 && !sel.has_any) {
					System.out.println("ControlSchemeManager: Ambiguous case detected, please refine selection.");
				}
			} else {
				// If a specific scheme is selected
				sel.devices = this.schemes.get(id).compatible(this.inputs);
				if (sel.devices != null && sel.devices.length > 0) {
					if (sel.has_any) {
						// Shut down the previously active scheme
						this.schemes.get(sel.prev_active_id).shutdown();
					}
					// Set up the selected scheme
					if (id != lastSelectedId) {
						this.schemes.get(id).setup(sel.devices);
						this.applied = this.schemes.get(id).getDesc();
						sel.prev_active_id = id;
						sel.prev_selected = id;
						sel.has_any = true;
					}
					lastSelectedId = id;
				}
			}
		}
	}
}
