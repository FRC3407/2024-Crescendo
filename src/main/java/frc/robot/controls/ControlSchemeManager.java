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

		/** Optional callback for debinding the scheduled bindings */
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
		 * @param description Description of the control scheme.
		 * @param compat      Compatibility function.
		 * @param setup       Setup function.
		 */
		public ControlScheme(String description, Compat_F compat, Setup_F setup) {
			this(description, compat, setup, null);
		}

		/**
		 * @param description Description of the control scheme.
		 * @param compat      Compatibility function.
		 * @param setup       Setup function.
		 * @param shutdown    Shutdown function.
		 */
		public ControlScheme(String description, Compat_F compat, Setup_F setup, Runnable shutdown) {
			this.compatibility = compat;
			this.setup = setup;
			this.shutdown = shutdown;
			;
			this.desc = description;
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
		 * @param requirements InputMap requirements.
		 */
		public AutomatedTester(InputMap... requirements) {
			this.requirements = requirements;
			this.buff = new InputDevice[requirements.length];
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
			int[] avail, requirements = new int[this.requirements.length]; // Look-up tables for inputs (param) and
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

			for (int i = 0; i < requirements.length; i++) {
				requirements[i] = i;
			}

			for (int i = found; i < avail.length; i++) { // For each remaining port:
				int p = avail[i];
				for (int r = found; r < requirements.length; r++) { // For each remaining requirement:
					if (this.requirements[requirements[r]].compatible(p)) { // If compatible:
						this.buff[requirements[r]] = inputs[p];
						if (i != found) {
							for (int q = i; q > found; q--) { // Start at i, work backwards until @ found
								avail[q] = avail[q - 1]; // Bump all values before i up an index
							}
						}
						if (r != found) {
							for (int q = r; q > found; q--) { // Same as above but for the requirements look-up table
								requirements[q] = requirements[q - 1];
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

	private final ArrayList<ControlSchemeBase> schemes = new ArrayList<>();
	private final InputDevice[] inputs = new InputDevice[DriverStation.kJoystickPorts]; // make static?
	private SendableChooser<Integer> options = new SendableChooser<>();
	private String applied = "None";
	private AmbiguousSolution ambiguousPreference = AmbiguousSolution.PREFER_COMPLEX;

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
	 * @param CSBase ControlSchemeBase to be added.
	 */
	public void addScheme(ControlSchemeBase CSBase) {
		this.options.addOption(CSBase.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(CSBase);
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility and setup
	 * functions.
	 * 
	 * @param description Description of the control scheme.
	 * @param compat      Compatibility function.
	 * @param setup       Setup function.
	 */
	public void addScheme(String description, ControlSchemeBase.Compat_F compat, ControlSchemeBase.Setup_F setup) {
		this.options.addOption(description, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(description, compat, setup));
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param description Description of the control scheme.
	 * @param compat      Compatibility function.
	 * @param setup       Setup function.
	 * @param shutdown    Shutdown function.
	 */
	public void addScheme(String description, ControlSchemeBase.Compat_F compat, ControlSchemeBase.Setup_F setup,
			Runnable shutdown) {
		this.options.addOption(description, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(description, compat, setup, shutdown));
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param description  Description of the control scheme.
	 * @param setup        Setup function.
	 * @param requirements InputMap requirements.
	 */
	public void addScheme(String description, ControlSchemeBase.Setup_F setup, InputMap... requirements) {
		this.addScheme(description, new AutomatedTester(requirements), setup);
	}

	/**
	 * Overloaded method to add a new control scheme with compatibility, setup, and
	 * shutdown functions.
	 * 
	 * @param description  Description of the control scheme.
	 * @param setup        Setup function.
	 * @param shutdown     Shutdown function.
	 * @param requirements InputMap requirements.
	 */
	public void addScheme(String description, ControlSchemeBase.Setup_F setup, Runnable shutdown,
			InputMap... requirements) {
		this.addScheme(description, new AutomatedTester(requirements), setup, shutdown);
	}

	/**
	 * Set the default control scheme.
	 * 
	 * @param CSBase ControlSchemeBase to be set as default.
	 */
	public void setDefault(ControlSchemeBase CSBase) {
		this.options.setDefaultOption(CSBase.getDesc(), Integer.valueOf(this.schemes.size()));
		this.schemes.add(CSBase);
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new scheme.
	 * 
	 * @param description Description of the control scheme.
	 * @param compat      Compatibility function.
	 * @param setup       Setup function.
	 */
	public void setDefault(String description, ControlSchemeBase.Compat_F compat, ControlSchemeBase.Setup_F setup) {
		this.options.setDefaultOption(description, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(description, compat, setup));
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new scheme.
	 * 
	 * @param description Description of the control scheme.
	 * @param compat      Compatibility function.
	 * @param setup       Setup function.
	 * @param shutdown    Shutdown function.
	 */
	public void setDefault(String description, ControlSchemeBase.Compat_F compat, ControlSchemeBase.Setup_F setup,
			Runnable shutdown) {
		this.options.setDefaultOption(description, Integer.valueOf(this.schemes.size()));
		this.schemes.add(new ControlScheme(description, compat, setup, shutdown));
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new
	 * AutomatedTester.
	 * 
	 * @param description  Description of the control scheme.
	 * @param setup        Setup function.
	 * @param requirements InputMap requirements.
	 */
	public void setDefault(String description, ControlSchemeBase.Setup_F setup, InputMap... requirements) {
		this.setDefault(description, new AutomatedTester(requirements), setup);
	}

	/**
	 * Overloaded method to set the default control scheme by creating a new
	 * AutomatedTester.
	 * 
	 * @param description  Description of the control scheme.
	 * @param setup        Setup function.
	 * @param shutdown     Shutdown function.
	 * @param requirements InputMap requirements.
	 */
	public void setDefault(String description, ControlSchemeBase.Setup_F setup, Runnable shutdown,
			InputMap... requirements) {
		this.setDefault(description, new AutomatedTester(requirements), setup, shutdown);
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
	 * @param name Name of the control scheme selector.
	 */
	public void publishSelector(String name) {
		SmartDashboard.putData(name + "/Selector", this.options);
		SmartDashboard.putData(name, this);
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
	 * @param nt   Specific Network table to publish to.
	 * @param name Name of the control scheme selector.
	 */
	public void publishSelector(SenderNT nt, String name) {
		nt.putData(name + "/Selector", this.options);
		nt.putData(name, this);
	}

	/**
	 * Set the preference for handling ambiguous control scheme solutions.
	 * 
	 * @param preference AmbiguousSolution preference.
	 */
	public void setAmbiguousSolution(AmbiguousSolution preference) {
		this.ambiguousPreference = preference;
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

	private static int lastSelectedCompat = 10;
	private static int lastSelectedId = 10;

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
						switch (this.ambiguousPreference) {
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
