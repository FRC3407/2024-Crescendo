package frc.robot.controls;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class houses an advanced input framework that extends the general purpose WPILib input system.
 */
public class Input {

	/**
	 * Interface for supplying joystick values (doubles)
	 */
	public static interface AnalogSupplier extends DoubleSupplier {
		double get();
		@Override default double getAsDouble() { return get(); }
	}
	/**
	 * Interface for supplying button presses (booleans)
	 */
	public static interface DigitalSupplier extends BooleanSupplier {
		boolean get();
		@Override default boolean getAsBoolean() { return get(); }
	}

	/**
	 * AnalogSlewSupplier limits the rate of output change (between calls) to a maximum of that given in units/sec
	 */
	public static class AnalogSlewSupplier implements AnalogSupplier {

		private final SlewRateLimiter limit;
		private final DoubleSupplier source;

		public AnalogSlewSupplier(DoubleSupplier src) { this(src, Double.MAX_VALUE); }  // kind of pointless?
		public AnalogSlewSupplier(DoubleSupplier src, double mrate) {
			this.limit = new SlewRateLimiter(mrate, -mrate, src.getAsDouble());
			this.source = src;
		}

		@Override public double get() {
			return this.limit.calculate(this.source.getAsDouble());
		}

	}
	/**
	 * AnalogDeadzone applies a deadzone to an analog supplier, returning 0.0 if the input is within the deadzone (+/-)
	 */
	public static class AnalogDeadzone implements AnalogSupplier {
		
		private final DoubleSupplier source;
		private final double deadzone;

		public AnalogDeadzone(DoubleSupplier src, double dz) {
			this.source = src;
			this.deadzone = dz;
		}

		@Override public double get() {
			double i = this.source.getAsDouble();
			return Math.abs(i) > this.deadzone ? i : 0.0;
		}

	}
	/**
	 * AnalogScalar simply scales the input by a constant (multiplication)
	 */
	public static class AnalogScalar implements AnalogSupplier {

		private final DoubleSupplier source;
		private final double scale;

		public AnalogScalar(DoubleSupplier src, double scl) {
			this.source = src;
			this.scale = scl;
		}

		@Override public double get() {
			return this.source.getAsDouble() * this.scale;
		}

	}
	/**
	 * AnalogExponential acts a passthrough for a base supplier, and returns each input taken to the given power
	 */
	public static class AnalogExponential implements AnalogSupplier {

		private final DoubleSupplier source;
		private final double power;

		public AnalogExponential(DoubleSupplier src, double pow) {
			this.source = src;
			this.power = pow;
		}

		@Override public double get() {
			double i = this.source.getAsDouble();
			return Math.copySign(Math.pow(Math.abs(i), this.power), i);
		}

	}
	/**
	 * DriveInputSupplier allows for the application of a deadzone, rescale, and exponential power transformation on an input supplier - useful for driving contexts as the name suggests
	 */
	public static class DriveInputSupplier implements AnalogSupplier {
		
		private final DoubleSupplier source;
		private final double exp_pow;
		private final double scale;		// when the input range is -1 to 1 this acts as a 'maximum output' modifier
		private final double deadband;

		public DriveInputSupplier(DoubleSupplier src, double deadband, double scale, double exp) {
			this.source = src;
			this.exp_pow = exp;
			this.scale = scale;
			this.deadband = deadband;
		}
		public DriveInputSupplier(DoubleSupplier src, double deadband, double scale) {
			this(src, deadband, scale, 1.0);
		}
		public DriveInputSupplier(DoubleSupplier src, double deadband) {
			this(src, deadband, 1.0, 1.0);
		}

		public static double compute(double x, double d) { return compute(x, d, 1.0, 1.0); }
		public static double compute(double x, double d, double m) { return compute(x, d, m, 1.0); }
		public static double compute(double x, double d, double m, double p) {
			double ax = Math.abs(x);
			if(ax < d) { return 0; }
			double dp = d;
			if(p != 1.0) {
				dp = Math.pow(d, p);
				ax = Math.pow(x, p);
			}
			double n = dp / (1.0 - dp);
			return Math.signum(x) * m * ((1.0 + n) * ax - n);
		}

		@Override public double get() {
			return compute(this.source.getAsDouble(), this.deadband, this.scale, this.exp_pow);
		}

	}


	/**
	 * InputDevice extends all {@link GenericHID} functionality, but adds some advanced trigger and POVButton mapping features.
	 * It is also required for all Analog and Digital enum interfaces defined below. 
	 */
	public static class InputDevice extends GenericHID {

		private PovButton[] buttons = null;
		private boolean verifyInfo() {
			if(super.isConnected()) {
				int digital_count = super.getButtonCount() + (super.getPOVCount() * 4);
				if(this.buttons == null || this.buttons.length != digital_count) {
					this.buttons = new PovButton[digital_count];
					return true;
				}
			} else if(!super.isConnected() && this.buttons != null) {
				this.buttons = null;
				return false;
			}
			return this.buttons != null;
		}

		public InputDevice(int p) { 
			super(p); 
			this.verifyInfo();
		}

		@Override
		public boolean getRawButton(int button) {
			int _bc = super.getButtonCount();
			if((button > _bc) && (button <= _bc + (super.getPOVCount() * 4))) {
				button = button - _bc;
				return (super.getPOV((button-1) / 4) / 90.0 + 1) == button;
			} else {
				return super.getRawButton(button);
			}
		}

		public PovButton getTrigger(int button) {  // returns dummy if joystick not connected
			if(this.verifyInfo()) {
				if(this.buttons[button-1] == null) {
					this.buttons[button-1] = new PovButton(this, button);
				}
				return this.buttons[button-1];
			}
			return PovButton.dummy;
		}

		public Trigger connectionTrigger() {
			return new Trigger(()->{ return super.isConnected(); });
		}


		public static void logDevice(GenericHID i) { logDevice(i.getPort()); }
		public static void logDevice(int p) {
			if(DriverStation.isJoystickConnected(p)) {
				System.out.println("\tDS Port[" + p + "] >> Axis count: " +
					DriverStation.getStickAxisCount(p) + ", Button count: " +
					DriverStation.getStickButtonCount(p) + ", POV count: " +
					DriverStation.getStickPOVCount(p));
			}
		}
		public static void logConnections() {
			System.out.println("Connected DS Inputs:");
			for(int i = 0; i < DriverStation.kJoystickPorts; i++) {
				if(DriverStation.isJoystickConnected(i)) {
					logDevice(i);
				}
			}
		}


	}

	/**
	 * PovButton remaps all pov's on a joystick to additional button indices past those normally used. Functionality and use is
	 * the same as a normal joystick button. 
	 */
	public static class PovButton extends Trigger implements DigitalSupplier {

		public static class Supplier implements DigitalSupplier {
			private final int port, button;
			private final boolean use_pov;
			public Supplier(int port, int button) {
				this.port = port;
				int _bc = DriverStation.getStickButtonCount(this.port), _pc = DriverStation.getStickPOVCount(this.port);
				this.use_pov = ((button > _bc) && (button <= _bc + (_pc * 4)));
				if (this.use_pov) { 
					this.button = button - _bc;
				} else { 
					this.button = button;
				}
			}
			@Override public boolean get() {
				if (this.use_pov) {
					return (DriverStation.getStickPOV(this.port, (this.button - 1) / 4) / 90.0 + 1) == this.button;
				} else {
					return DriverStation.getStickButton(this.port, this.button);
				}
			}
		}

		public static final PovButton dummy = new PovButton();

		private PovButton() {
			super(()->false);
		}
		public PovButton(GenericHID device, int button) { this(device.getPort(), button); }
		public PovButton(int port, int button) {
			super(new PovButton.Supplier(port, button));
		}

		@Override public boolean get() {
			return super.getAsBoolean();
		}


	}

	/**
	 * AnalogMap defines all functions available to Analog enums (defined below)
	 */
	public static interface AnalogMap {
		int getValue();
		int getTotal();
		default boolean compatible(GenericHID i) { return i.getAxisCount() == this.getTotal(); }
		default boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == this.getTotal(); }
		default double getValueOf(GenericHID i) {
			if(this.compatible(i)) {
				return i.getRawAxis(this.getValue());
			}
			return 0.0;
		}
		default double getValueOf(int p) {
			if(this.compatible(p)) {
				return DriverStation.getStickAxis(p, this.getValue());
			}
			return 0.0;
		}
		default AnalogSupplier getSupplier(InputDevice i) {
			if(this.compatible(i)) {
				return ()->i.getRawAxis(this.getValue());
			}
			return ()->0.0;
		}
		default AnalogSupplier getSupplier(int p) {
			if(this.compatible(p)) {
				return ()->DriverStation.getStickAxis(p, this.getValue());
			}
			return ()->0.0;
		}
		default AnalogSlewSupplier getLimitedSupplier(InputDevice i, double mrate) {
			if(this.compatible(i)) {
				return new AnalogSlewSupplier(()->i.getRawAxis(this.getValue()), mrate);
			}
			return new AnalogSlewSupplier(()->0.0);
		}
		default AnalogSlewSupplier getLimitedSupplier(int p, double mrate) {
			if(this.compatible(p)) {
				return new AnalogSlewSupplier(()->DriverStation.getStickAxis(p, this.getValue()), mrate);
			}
			return new AnalogSlewSupplier(()->0.0);
		}
		default AnalogExponential getExponentialSupplier(InputDevice i, double pow) {
			if(this.compatible(i)) {
				return new AnalogExponential(()->i.getRawAxis(this.getValue()), pow);
			}
			return new AnalogExponential(()->0.0, 1.0);
		}
		default AnalogExponential getExponentialSupplier(int p, double pow) {
			if(this.compatible(p)) {
				return new AnalogExponential(()->DriverStation.getStickAxis(p, this.getValue()), pow);
			}
			return new AnalogExponential(()->0.0, 1.0);
		}
		default AnalogSupplier getExponentialLimitedSupplier(InputDevice i, double mrate, double pow) {
			if(this.compatible(i)) {
				return new AnalogSlewSupplier(new AnalogExponential(()->i.getRawAxis(this.getValue()), pow), mrate);
			}
			return ()->0.0;
		}
		default AnalogSupplier getExponentialLimitedSupplier(int p, double mrate, double pow) {
			if(this.compatible(p)) {
				return new AnalogSlewSupplier(new AnalogExponential(()->DriverStation.getStickAxis(p, this.getValue()), pow), mrate);
			}
			return ()->0.0;
		}
		default AnalogSupplier getDriveInputSupplier(InputDevice i, double deadzone, double scale, double exp) {
			if(this.compatible(i)) {
				return new DriveInputSupplier(()->i.getRawAxis(this.getValue()), deadzone, scale, exp);
			}
			return ()->0.0;
		}
		default AnalogSupplier getDriveInputSupplier(int p, double deadzone, double scale, double exp) {
			if(this.compatible(p)) {
				return new DriveInputSupplier(()->DriverStation.getStickAxis(p, this.getValue()), deadzone, scale, exp);
			}
			return ()->0.0;
		}
	}
	/**
	 * DigitalMap defines all functions available to Digital enums defined below. Provides integration with PovButton when available. 
	 */
	public static interface DigitalMap {
		int getValue();
		int getTotal();
		default boolean compatible(GenericHID i) {
			return i.getButtonCount() + (i.getPOVCount()*4) == this.getTotal();
		}
		default boolean compatible(int p) {
			return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == this.getTotal();
		}
		default boolean isPovBindOf(GenericHID i) {
			if(this.compatible(i)) {
				return this.getValue() > i.getButtonCount();
			}
			return false;
		}
		default boolean isPovBindOf(int p) {
			if(this.compatible(p)) {
				return this.getValue() > DriverStation.getStickPOVCount(p);
			}
			return false;
		}
		default int getPovBindOf(GenericHID i) {
			if(this.compatible(i) && this.isPovBindOf(i)) {
				return (this.getValue() - i.getButtonCount())/4;
			}
			return -1;
		}
		default int getPovBindOf(int p) {
			if(this.compatible(p) && this.isPovBindOf(p)) {
				return (this.getValue() - DriverStation.getStickButtonCount(p))/4;
			}
			return -1;
		}
		default PovButton getCallbackFrom(InputDevice i) {
			if(this.compatible(i)) {
				return i.getTrigger(this.getValue());
			}
			return PovButton.dummy;
		}
		default PovButton getCallbackFrom(GenericHID i) {
			if(this.compatible(i)) {
				return new PovButton(i, this.getValue());
			}
			return PovButton.dummy;
		}
		default PovButton getCallbackFrom(int p) {
			if(this.compatible(p)) {
				return new PovButton(p, this.getValue());
			}
			return PovButton.dummy;
		}
		// default ToggleTrigger getToggleFrom(InputDevice i) {
		// 	return new ToggleTrigger(getCallbackFrom(i));
		// }
		// default ToggleTrigger getToggleFrom(GenericHID i) {
		// 	return new ToggleTrigger(getCallbackFrom(i));
		// }
		// default ToggleTrigger getToggleFrom(int p) {
		// 	return new ToggleTrigger(getCallbackFrom(p));
		// }
		default boolean getValueOf(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return i.getRawButton(this.getValue());
			}
			return false;
		}
		default boolean getValueOf(int p) {
			if(this.isPovBindOf(p)) {
				return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return DriverStation.getStickButton(p, this.getValue());
			}
			return false;
		}
		default boolean getPressedValueOf(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return i.getRawButtonPressed(this.getValue());
			}
			return false;
		}
		default boolean getPressedValueOf(int p) {
			if(this.isPovBindOf(p)) {
				return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return DriverStation.getStickButtonPressed(p, this.getValue());
			}
			return false;
		}
		default boolean getReleasedValueOf(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return i.getRawButtonReleased(this.getValue());
			}
			return false;
		}
		default boolean getReleasedValueOf(int p) {
			if(this.isPovBindOf(p)) {
				return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return DriverStation.getStickButtonReleased(p, this.getValue());
			}
			return false;
		}
		default DigitalSupplier getSupplier(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return ()->i.getRawButton(this.getValue());
			}
			return ()->false;
		}
		default DigitalSupplier getSupplier(int p) {
			if(this.isPovBindOf(p)) {
				return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return ()->DriverStation.getStickButton(p, this.getValue());
			}
			return ()->false;
		}
		default DigitalSupplier getPressedSupplier(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return ()->i.getRawButtonPressed(this.getValue());
			}
			return ()->false;
		}
		default DigitalSupplier getPressedSupplier(int p) {
			if(this.isPovBindOf(p)) {
				return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return ()->DriverStation.getStickButtonPressed(p, this.getValue());
			}
			return ()->false;
		}
		default DigitalSupplier getReleasedSupplier(GenericHID i) {
			if(this.isPovBindOf(i)) {
				return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
			} else if(this.compatible(i)) {
				return ()->i.getRawButtonReleased(this.getValue());
			}
			return ()->false;
		}
		default DigitalSupplier getReleasedSupplier(int p) {
			if(this.isPovBindOf(p)) {
				return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
			} else if(this.compatible(p)) {
				return ()->DriverStation.getStickButtonReleased(p, this.getValue());
			}
			return ()->false;
		}
	}
	/**
	 * InputMap defines all functions available to Input classes that contain an AnalogMap and DigitalMap
	 */
	public static abstract class InputMap {
		public abstract boolean compatible(GenericHID i);
		public abstract boolean compatible(int p);

		protected boolean compat(GenericHID i, AnalogMap a, DigitalMap d) {
			return a.compatible(i) && d.compatible(i);
		}
		protected boolean compat(int p, AnalogMap a, DigitalMap d) {
			return a.compatible(p) && d.compatible(p);
		}
	}


	/**
	 * All button and axis indices for an Xbox controller
	 */
	public static class Xbox extends InputMap {
		public static enum Analog implements AnalogMap {
			LX(0), RX(4), LY(1), RY(5), LT(2), RT(3), 
			TOTAL(6);
		
			public final int value;
			private Analog(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}
		public static enum Digital implements DigitalMap {
			LB(5), RB(6), LS(9), RS(10),
			A(1), B(2), X(3), Y(4),
			BACK(7), START(8),
			DT(11), DR(12), DB(13), DL(14),  // Dpad buttons (only valid with PovButton objects)
			TOTAL(14);
		
			public final int value;
			private Digital(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private Xbox() {}
		public static final Xbox Map = new Xbox();

		public boolean compatible(GenericHID i)
			{ return super.compat(i, Analog.TOTAL, Digital.TOTAL); }
		public boolean compatible(int p)
			{ return super.compat(p, Analog.TOTAL, Digital.TOTAL); }
	}
	/**
	 * All button and axis indices for a Playstation controller
	 */
	public static class PlayStation extends InputMap {
		public static enum Analog implements AnalogMap {
			LX(0), LY(1), RX(2), RY(5), LT(3), RT(4),
			TOTAL(6);

			public final int value;
			private Analog(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}
		public static enum Digital implements DigitalMap {
			SQR(1), X(2), O(3), TRI(4),     // square, cross, circle, triangle
			LB(5), RB(6), L2(7), R2(8),     // right-bumper, left-bumper,   left-trigger, right-trigger (button mode)
			SHARE(9), OPT(10), PS(13),      // share, options, ps button
			TOUCH(14), LS(11), RS(12),      // touchpad, left-stick, right-stick
			TOTAL(14);

			public final int value;
			private Digital(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private PlayStation() {}
		public static final PlayStation Map = new PlayStation();

		public boolean compatible(GenericHID i)
			{ return super.compat(i, Analog.TOTAL, Digital.TOTAL); }
		public boolean compatible(int p)
			{ return super.compat(p, Analog.TOTAL, Digital.TOTAL); }
	}
	/**
	 * All button and axis indices for an Attack3 joystick
	 */
	public static class Attack3 extends InputMap {
		public static enum Analog  implements AnalogMap {
			X(0), Y(1), S(2),   // ~ X-Axis, Y-Axis, Slider thing on the bottom
			TOTAL(3);

			public final int value;
			private Analog(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}
		public static enum Digital implements DigitalMap {
			TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
			B1(6), B2(7), B3(8), B4(9), B5(10), B6(11),     // ~ buttons on the base of the joystick (labeled)
			TOTAL(11);

			public final int value;
			private Digital(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private Attack3() {}
		public static final Attack3 Map = new Attack3();
		public boolean compatible(GenericHID i)
			{ return super.compat(i, Analog.TOTAL, Digital.TOTAL); }
		public boolean compatible(int p)
			{ return super.compat(p, Analog.TOTAL, Digital.TOTAL); }
	}
	/**
	 * All button and axis indices for an Extreme3D joystick
	 */
	public static class Extreme3d extends InputMap {
		public static enum Analog  implements AnalogMap {
			X(0), Y(1), Z(2), S(3),     // x-axis, y-axis, swivell-axis, slider-axis
			TOTAL(4);

			public final int value;
			private Analog(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}
		public static enum Digital implements DigitalMap {
			TRI(1), SIDE(2), TLB(3), TRB(4), TLT(5), TRT(6),    // trigger, side, top-left-bottom, top-right-bottom, top-left-top, top-right-top
			B7(7), B8(8), B9(9), B10(10), B11(11), B12(12),     // as printed on the actual joystick
			TOTAL(12);

			public final int value;
			private Digital(int value) { this.value = value; }

			public int getValue() { return this.value; }
			public int getTotal() { return TOTAL.value; }
		}

		private Extreme3d() {}
		public static final Extreme3d Map = new Extreme3d();

		public boolean compatible(GenericHID i)
			{ return super.compat(i, Analog.TOTAL, Digital.TOTAL); }
		public boolean compatible(int p)
			{ return super.compat(p, Analog.TOTAL, Digital.TOTAL); }
	}

}