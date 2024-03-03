package frc.robot.controls;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Input.ButtonBox;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoGoCommand;
import frc.robot.commands.AutoSelector;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FlingCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.Robot;
import frc.robot.controls.ControlSchemeManager.AmbiguousSolution;
import frc.robot.controls.ControlSchemeManager.AutomatedTester;
import frc.robot.controls.ControlSchemeManager.ControlScheme;
import frc.robot.controls.ControlSchemeManager.ControlSchemeBase;
import frc.robot.controls.Input.Attack3;
import frc.robot.controls.Input.InputDevice;
import frc.robot.controls.Input.InputMap;
import frc.robot.controls.Input.PlayStation;
import frc.robot.controls.Input.Xbox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FloorIntake;
import frc.utils.TriggerRunnable;
import frc.robot.subsystems.Flinger;

public final class Controls {

	public static final Flinger m_flinger = new Flinger();
	public static final FloorIntake m_intake = new FloorIntake();
	public final static DriveSubsystem m_driveTrain = new DriveSubsystem();

	// Can be used to create situations where a specific list of schemes are used
	// rather than all of them
	public static enum FeatureLevel {
		ALLSCHEMES
	}

	// Types of drives this robot can use, only really applicable to non-swerve
	// robots that have the choice of either arcade or tank (not this robot)
	public static enum DriveMode {
		DEFAULTSWERVE
	}

	// By default, uses all schemes in getAllSchemes
	public static final FeatureLevel DEFAULT_FEATURE_LEVEL = FeatureLevel.ALLSCHEMES;
	// By default, uses swerve for drive
	public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.DEFAULTSWERVE;
	// A list of TriggerRunnables, cleared in deScheduleCommands whenever a new
	// schemed is used
	// to prevent duplicate commands
	// All TriggerRunnables are polled in pollCommands, allowing their conditions to
	// be checked
	// and their commands to be ran
	private static ArrayList<TriggerRunnable> triggerList = new ArrayList<TriggerRunnable>();

	/**
	 * @param robot
	 * @return A list of all schemes that the robot can use (new schemes must be
	 *         added to this list)
	 */
	private static ControlScheme[] getAllSchemes(Robot robot) {
		return new ControlScheme[] {
				buildScheme("Single Xbox", (InputDevice... i) -> singleXbox(robot, DEFAULT_DRIVE_MODE, i), Xbox.Map),
				buildScheme("Single PlayStation", (InputDevice... i) -> singlePlayStation(robot, DEFAULT_DRIVE_MODE, i),
						PlayStation.Map),
				buildScheme("Dual Xbox", (InputDevice... i) -> dualXbox(robot, DEFAULT_DRIVE_MODE, i), Xbox.Map,
						Xbox.Map),
				buildScheme("Simple Arcade", (InputDevice... i) -> arcadeBoardSimple(robot, DEFAULT_DRIVE_MODE, i),
						Attack3.Map, Attack3.Map),
				buildScheme("Full Arcade", (InputDevice... i) -> arcadeBoardFull(robot, DEFAULT_DRIVE_MODE, i),
						Attack3.Map, Attack3.Map, ButtonBox.Map),
				buildScheme("Competition Board",
						(InputDevice... i) -> fullCompetitionBoard(robot, DEFAULT_DRIVE_MODE, i),
						Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map)
		};
	}

	// Example of a specific list of schemes to use, this list of schemes on has
	// just the
	// "Full Arcade" scheme in it

	// private static ControlScheme[] getCompetitionSchemes(Robot robot) {
	// ControlScheme defaultSwerve = buildScheme("Full Arcade",
	// (InputDevice... i) -> arcadeBoardFull(robot, DEFAULT_DRIVE_MODE, i),
	// Attack3.Map, Attack3.Map, ButtonBox.Map);
	// return DEFAULT_DRIVE_MODE == DriveMode.DEFAULTSWERVE ? new ControlScheme[] {
	// defaultSwerve }
	// : new ControlScheme[] { null };
	// }

	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager) {
		return setupControls(robot, manager, null);
	}

	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager, FeatureLevel features) {
		if (manager == null) {
			manager = new ControlSchemeManager();
		}
		if (features == null) {
			features = DEFAULT_FEATURE_LEVEL;
		}
		switch (features) {
			case ALLSCHEMES: {
				ControlScheme[] schemes = getAllSchemes(robot);
				for (ControlScheme sch : schemes) {
					manager.addScheme(sch);
				}
				break;
			}

			// Example case for specific feature group, this feature level only uses schemes
			// from getCompetitionSchemes

			// case COMPETITION: {
			// ControlScheme[] schemes = getCompetitionSchemes(robot);
			// manager.setDefault(schemes[0]);
			// for (int i = 1; i < schemes.length; i++) {
			// manager.addScheme(schemes[i]);
			// }
			// break;
			// }
		}
		manager.setAmbiguousSolution(AmbiguousSolution.PREFER_COMPLEX);
		manager.publishSelector("Control Scheme");
		return manager;
	}

	/**
	 * @param name  The name of the new ControlScheme
	 * @param setup
	 * @param reqs
	 * @return New ControlScheme with the parameters specified
	 */
	private static ControlScheme buildScheme(String name, ControlSchemeBase.Setup_F setup, InputMap... reqs) {
		return new ControlScheme(name, new AutomatedTester(reqs), setup);
	}

	/**
	 * Removes all default commands and clears the triggerList, used in beggining of
	 * every control scheme to prevent duplication of triggers & commands
	 */
	private static void deScheduleCommands() {
		m_driveTrain.removeDefaultCommand();
		m_flinger.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		triggerList.clear();
	}

	/**
	 * Polls all TriggerRunnables within triggerList, if a TriggerRunnable's
	 * condition is true, it runs it's command
	 */
	public static void pollCommands() {
		for (TriggerRunnable triggerRunnable : triggerList) {
			triggerRunnable.poll();
		}
	}

	// Changed by AutoSelector.java
	public static Command selectedAutoCommand = new AutoGoCommand(m_driveTrain);

	/**
	 * @return The selected auto command
	 */
	public static Command getAutonomousCommand() {
		return selectedAutoCommand;
	}

	// Uses a Single Xbox Controller
	private static void singleXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice controller = inputs[0];
		Command drive_control = new DriveCommand(m_driveTrain,
				Xbox.Analog.RX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.RY.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.LX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.LT.getDriveInputSupplier(controller, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Digital.B.getSupplier(controller));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> Xbox.Analog.RT.getValueOf(controller) >= OIConstants.kTriggerDeadband,
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> Xbox.Digital.RB.getValueOf(controller),
				new IntakeCommand(m_flinger, m_intake)));
	}

	// Uses a Single PlayStation controller
	private static void singlePlayStation(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice controller = inputs[0];
		Command drive_control = new DriveCommand(m_driveTrain,
				PlayStation.Analog.RX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.RY.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.LX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.LT.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Digital.O.getSupplier(controller));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> PlayStation.Analog.RT.getValueOf(controller) >= OIConstants.kTriggerDeadband,
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> PlayStation.Digital.RB.getValueOf(controller),
				new IntakeCommand(m_flinger, m_intake)));
	}

	// Uses two Xbox controllers
	private static void dualXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice controller1 = inputs[0];
		InputDevice controller2 = inputs[1];
		Command drive_control = new DriveCommand(m_driveTrain,
				Xbox.Analog.RX.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.RY.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.LX.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.LT.getDriveInputSupplier(controller1, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Digital.B.getSupplier(controller1));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> Xbox.Analog.RT.getValueOf(controller1) >= OIConstants.kTriggerDeadband,
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> Xbox.Digital.RB.getValueOf(controller1),
				new IntakeCommand(m_flinger, m_intake)));
	}

	// Uses two Attack Joysticks
	private static void arcadeBoardSimple(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> Attack3.Digital.TRI.getValueOf(rstick),
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> Attack3.Digital.TB.getValueOf(rstick),
				new IntakeCommand(m_flinger, m_intake)));
	}

	// Uses two Attack Joysticks and the Button Box
	private static void arcadeBoardFull(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> Attack3.Digital.TRI.getValueOf(rstick),
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> Attack3.Digital.TB.getValueOf(rstick),
				new IntakeCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onToggle, // Auto Select
				() -> ButtonBox.Digital.S1.getValueOf(bbox),
				new AutoSelector(() -> ButtonBox.Digital.S1.getValueOf(bbox),
						() -> ButtonBox.Digital.S2.getValueOf(bbox))));

	}

	// Uses two Attack Joysticks, the Button Box, and an Xbox controller
	private static void fullCompetitionBoard(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		deScheduleCommands();
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		InputDevice controller = inputs[3];
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Fling
				() -> Attack3.Digital.TRI.getValueOf(rstick),
				new FlingCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onTrue, // Intake
				() -> Attack3.Digital.TB.getValueOf(rstick),
				new IntakeCommand(m_flinger, m_intake)));
		triggerList.add(new TriggerRunnable(TriggerRunnable.LoopType.onToggle, // Auto Select
				() -> ButtonBox.Digital.S1.getValueOf(bbox),
				new AutoSelector(() -> ButtonBox.Digital.S1.getValueOf(bbox),
						() -> ButtonBox.Digital.S2.getValueOf(bbox))));
	}

	// How to write a new control scheme
	// Use the following shell:

	// private static void nameOfScheme(Robot robot, DriveMode drivemode,
	// InputDevice... inputs)
	// {
	// deScheduleCommands();
	// InputDevice nameOfDevice = inputs[port]; //Step 1
	// Command drive_control = new DriveCommand(m_driveTrain, //Step 2
	// inputDeviceType.Analog/Digital.button.getDriveInputSupplier(controller, 0,
	// 1.0, 1.0),
	// inputDeviceType.Analog/Digital.button.getDriveInputSupplier(controller, 0,
	// 1.0, 1.0),
	// inputDeviceType.Analog/Digital.button.getDriveInputSupplier(controller, 0,
	// 1.0, 1.0),
	// inputDeviceType.Analog/Digital.button.getSupplier(controller),
	// inputDeviceType.Analog/Digital.button.getSupplier(controller));
	// m_driveTrain.setDefaultCommand(drive_control);
	// triggerList.add(new TriggerRunnable(LoopType, //Step 3
	// buttonSupplier,
	// command));
	// }

	// Step 1: For each type of InputDevice you want to use add a new InputDevice,
	// name it according to the
	// type of device, then set the port to the port you want

	// Step 2: For the drive change the inputDeviceTypes and buttons to what you
	// want
	// based on your device type's input map in Input.java
	// Then change controller to the type of controller you are getting the input
	// from
	// Repeat for each subsystem the uses a defaultCommand

	// Step 3: For each individual command that is being run, add a copy of the
	// triggerList.add code
	// Replace the loopType with the type of loop you would like, most of the types
	// from the wpilib
	// trigger class are usable such as onTrue and whileFalse, the available types
	// can be seen in
	// TriggerRunnable.LoopType
	// Replace buttonSupplier with the supplier you would like to use, to convert a
	// value function into
	// a supplier, use a lambda expression, for example: () ->
	// Attack3.Digital.TRI.getValueOf(rstick)
	// this would turn the getValueOf function for the Attack 3 trigger into a
	// supplier rather than just
	// a value function
	// Replace command wih the type of command you would like to run, wether it be a
	// new command or
	// one from the commands folder
}