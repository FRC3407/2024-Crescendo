package frc.robot.controls;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ButtonBox;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class Controls {

	public static final Shooter m_shooter = new Shooter();
	public static final Intake m_intake = new Intake();
	public final static DriveSubsystem m_driveTrain = new DriveSubsystem();

	public static enum FeatureLevel {
		TESTING,
		COMPETITION
	}

	public static enum DriveMode {
		DEFAULTSWERVE
	}

	public static final FeatureLevel DEFAULT_FEATURE_LEVEL = FeatureLevel.TESTING;
	public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.DEFAULTSWERVE;

	private static Command[] active_commands;

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
				buildScheme("Competition Board", (InputDevice... i) -> competitionBoard(robot, DEFAULT_DRIVE_MODE, i),
						Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map)
		};
	}

	private static ControlScheme[] getCompetitionSchemes(Robot robot) {
		ControlScheme
		// tank = buildScheme("Competition Controls (Tank Drive)", (InputDevice...
		// i)->competitionBoard(robot, DriveMode.TANK, i), Attack3.Map, Attack3.Map,
		// ButtonBox.Map, Xbox.Map),
		// arcade = buildScheme("Competition Controls (Arcade Drive)", (InputDevice...
		// i)->competitionBoard(robot, DriveMode.ARCADE, i), Attack3.Map, Attack3.Map,
		// ButtonBox.Map, Xbox.Map);
		defaultSwerve = buildScheme("Competition Controls (Default Swerve)",
				(InputDevice... i) -> competitionBoard(robot, DriveMode.DEFAULTSWERVE, i), Attack3.Map, Attack3.Map,
				ButtonBox.Map, Xbox.Map);
		return DEFAULT_DRIVE_MODE == DriveMode.DEFAULTSWERVE ? new ControlScheme[] { defaultSwerve }
				: new ControlScheme[] { null };
	}

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
			case TESTING: {
				ControlScheme[] schemes = getAllSchemes(robot);
				for (ControlScheme sch : schemes) {
					manager.addScheme(sch);
				}
				break;
			}
			case COMPETITION: {
				ControlScheme[] schemes = getCompetitionSchemes(robot);
				manager.setDefault(schemes[0]);
				for (int i = 1; i < schemes.length; i++) {
					manager.addScheme(schemes[i]);
				}
				break;
			}
		}
		manager.setAmbiguousSolution(AmbiguousSolution.PREFER_COMPLEX);
		manager.publishSelector("Control Scheme");
		return manager;
	}

	private static ControlScheme buildScheme(String name, ControlSchemeBase.Setup_F setup, InputMap... reqs) {
		return new ControlScheme(name, new AutomatedTester(reqs), setup, Controls::deScheduleActive);
	}

	private static void deScheduleActive() {
		for (Command c : active_commands) {
			c.cancel();
		}
	}

	// single xbox controller
	private static void singleXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice controller = inputs[0];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				Xbox.Analog.RX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.RY.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.LX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				Xbox.Analog.LT.getDriveInputSupplier(controller, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Analog.RT.getDriveInputSupplier(controller, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Digital.A.getSupplier(controller),
				Xbox.Digital.B.getSupplier(controller));
		// Command path = new PathCommand(m_driveTrain,)
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				Xbox.Digital.LB.getSupplier(controller));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				Xbox.Digital.RB.getSupplier(controller),
				Xbox.Digital.LB.getSupplier(controller));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Single Xbox Control Scheme Registered");
	}

	// single PlayStation controller
	private static void singlePlayStation(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice controller = inputs[0];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				PlayStation.Analog.RX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.RY.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.LX.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.LT.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Analog.RT.getDriveInputSupplier(controller, 0, 1.0, 1.0),
				PlayStation.Digital.X.getSupplier(controller),
				PlayStation.Digital.O.getSupplier(controller));
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				PlayStation.Digital.LB.getSupplier(controller));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				PlayStation.Digital.RB.getSupplier(controller),
				PlayStation.Digital.LB.getSupplier(controller));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Single PlayStation Control Scheme Registered");
	}

	// dual xbox controllers
	private static void dualXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice controller1 = inputs[0];
		InputDevice controller2 = inputs[1];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				Xbox.Analog.RX.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.RY.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.LX.getDriveInputSupplier(controller1, 0, 1.0, 1.0),
				Xbox.Analog.LT.getDriveInputSupplier(controller1, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Analog.RT.getDriveInputSupplier(controller1, OIConstants.kTriggerDeadband, 1.0, 1.0),
				Xbox.Digital.A.getSupplier(controller1),
				Xbox.Digital.B.getSupplier(controller1));
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				Xbox.Digital.LB.getSupplier(controller2));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				Xbox.Digital.RB.getSupplier(controller2),
				Xbox.Digital.LB.getSupplier(controller2));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Dual Xbox Control Scheme Registered");
	}

	// simple 2-joystick arcade board
	private static void arcadeBoardSimple(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.TRI.getSupplier(rstick),
				Attack3.Digital.B2.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				Attack3.Digital.B1.getSupplier(lstick));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				Attack3.Digital.B1.getSupplier(rstick),
				Attack3.Digital.B1.getSupplier(lstick));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Arcade Board Simple Control Scheme Registered");
	}

	// 2 joysticks plus the buttonbox
	private static void arcadeBoardFull(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.TRI.getSupplier(rstick),
				Attack3.Digital.B2.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				Attack3.Digital.B1.getSupplier(lstick));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				Attack3.Digital.B1.getSupplier(rstick),
				Attack3.Digital.B1.getSupplier(lstick));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Arcade Board Full Control Scheme Registered");
	}

	// full control board plus an extra controller
	private static void competitionBoard(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		InputDevice controller = inputs[3];
		m_driveTrain.removeDefaultCommand();
		m_shooter.removeDefaultCommand();
		m_intake.removeDefaultCommand();
		Command drive_control = new DriveCommand(m_driveTrain,
				Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
				Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
				Attack3.Digital.TRI.getSupplier(lstick),
				Attack3.Digital.TRI.getSupplier(rstick),
				Attack3.Digital.B2.getSupplier(lstick),
				Attack3.Digital.B2.getSupplier(rstick));
		m_driveTrain.setDefaultCommand(drive_control);
		Command intake_control = new IntakeCommand(m_intake,
				Attack3.Digital.B1.getSupplier(lstick));
		m_intake.setDefaultCommand(intake_control);
		Command shooter_control = new ShootCommand(m_shooter,
				Attack3.Digital.B1.getSupplier(rstick),
				Attack3.Digital.B1.getSupplier(lstick));
		m_shooter.setDefaultCommand(shooter_control);
		System.out.println("Competition Board Control Scheme Registered");
	}
}
