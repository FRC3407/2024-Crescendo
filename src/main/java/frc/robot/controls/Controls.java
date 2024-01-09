package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ButtonBox;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.controls.ControlSchemeManager.AmbiguousSolution;
import frc.robot.controls.ControlSchemeManager.AutomatedTester;
import frc.robot.controls.ControlSchemeManager.ControlScheme;
import frc.robot.controls.ControlSchemeManager.ControlSchemeBase;
import frc.robot.controls.Input.Attack3;
import frc.robot.controls.Input.InputDevice;
import frc.robot.controls.Input.InputMap;
import frc.robot.controls.Input.Xbox;


public final class Controls {

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
		return new ControlScheme[]{
			buildScheme("Single Xbox",		(InputDevice... i)->singleXbox(robot, DEFAULT_DRIVE_MODE, i),			Xbox.Map),
			buildScheme("Dual Xbox",			(InputDevice... i)->dualXbox(robot, DEFAULT_DRIVE_MODE, i),				Xbox.Map, Xbox.Map),
			buildScheme("Simple Arcade",		(InputDevice... i)->arcadeBoardSimple(robot, DEFAULT_DRIVE_MODE, i),	Attack3.Map, Attack3.Map),
			buildScheme("Full Arcade",		(InputDevice... i)->arcadeBoardFull(robot, DEFAULT_DRIVE_MODE, i),		Attack3.Map, Attack3.Map, ButtonBox.Map),
			buildScheme("Competition Board",	(InputDevice... i)->competitionBoard(robot, DEFAULT_DRIVE_MODE, i),		Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map)
		};
	}

	private static ControlScheme[] getCompetitionSchemes(Robot robot) {
		ControlScheme
			// tank = buildScheme("Competition Controls (Tank Drive)",		(InputDevice... i)->competitionBoard(robot, DriveMode.TANK, i),		Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map),
			// arcade = buildScheme("Competition Controls (Arcade Drive)",	(InputDevice... i)->competitionBoard(robot, DriveMode.ARCADE, i),	Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map);
			defaultSwerve = buildScheme("Competition Controls (Default Swerve)",	(InputDevice... i)->competitionBoard(robot, DriveMode.DEFAULTSWERVE, i),	Attack3.Map, Attack3.Map, ButtonBox.Map, Xbox.Map);
		return DEFAULT_DRIVE_MODE == DriveMode.DEFAULTSWERVE ?
			new ControlScheme[]{defaultSwerve} : new ControlScheme[]{null};
	}

	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager) { return setupControls(robot, manager, null); }
	public static ControlSchemeManager setupControls(Robot robot, ControlSchemeManager manager, FeatureLevel features) {
		if(manager == null) { manager = new ControlSchemeManager(); }
		if(features == null) { features = DEFAULT_FEATURE_LEVEL; }
		switch(features) {
			case TESTING: {
				ControlScheme[] schemes = getAllSchemes(robot);
				for(ControlScheme sch : schemes) {
					manager.addScheme(sch);
				}
				break;
			}
			case COMPETITION: {
				ControlScheme[] schemes = getCompetitionSchemes(robot);
				manager.setDefault(schemes[0]);
				for(int i = 1; i < schemes.length; i++) {
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
		for(Command c : active_commands) {
			c.cancel();
		}
	}

	// single xbox controller
	private static void singleXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice controller = inputs[0];
		CommandBase drive_control = new DriveCommand(RobotContainer.m_robotDrive, 
			Xbox.Analog.LX.getDriveInputSupplier(controller, 0, -1.0, 1.0), 
			Xbox.Analog.LY.getDriveInputSupplier(controller, 0, -1.0, 1.0), 
			Xbox.Analog.RY.getDriveInputSupplier(controller, 0, -1.0, 1.0), 
			Xbox.Analog.LT.getSupplier(controller),
			Xbox.Analog.RT.getSupplier(controller));
		RobotContainer.m_robotDrive.setDefaultCommand(drive_control);
	}

	// dual xbox controllers
	private static void dualXbox(Robot robot, DriveMode drivemode, InputDevice... inputs) {	
		InputDevice controller1 = inputs[0];
		InputDevice controller2 = inputs[1];
		CommandBase drive_control = new DriveCommand(RobotContainer.m_robotDrive, 
			Xbox.Analog.LX.getDriveInputSupplier(controller2, 0, -1.0, 1.0), 
			Xbox.Analog.LY.getDriveInputSupplier(controller2, 0, -1.0, 1.0), 
			Xbox.Analog.RY.getDriveInputSupplier(controller2, 0, -1.0, 1.0), 
			Xbox.Analog.LT.getSupplier(controller2),
			Xbox.Analog.RT.getSupplier(controller2));
		RobotContainer.m_robotDrive.setDefaultCommand(drive_control);
	}

	// simple 2-joystick arcade board
	private static void arcadeBoardSimple(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		CommandBase drive_control = new DriveCommand(RobotContainer.m_robotDrive, 
			Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0), 
			Attack3.Analog.Y.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
			Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
			Attack3.Digital.TRI.getSupplier(lstick),
			Attack3.Digital.TRI.getSupplier(rstick));
		RobotContainer.m_robotDrive.setDefaultCommand(drive_control);
		System.out.println("controlSchemeRegistered");
	}

	// 2 joysticks plus the buttonbox
	private static void arcadeBoardFull(Robot robot, DriveMode drivemode, InputDevice... inputs) {
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		CommandBase drive_control = new DriveCommand(RobotContainer.m_robotDrive, 
			Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0), 
			Attack3.Analog.Y.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
			Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
			Attack3.Digital.TRI.getSupplier(lstick),
			Attack3.Digital.TRI.getSupplier(rstick));
		RobotContainer.m_robotDrive.setDefaultCommand(drive_control);
	}

	// full control board plus an extra controller
	private static void competitionBoard(Robot robot, DriveMode drivemode, InputDevice... inputs) {	
		InputDevice lstick = inputs[0];
		InputDevice rstick = inputs[1];
		InputDevice bbox = inputs[2];
		InputDevice controller = inputs[3];
		CommandBase drive_control = new DriveCommand(RobotContainer.m_robotDrive, 
			Attack3.Analog.X.getDriveInputSupplier(lstick, 0, 1.0, 1.0), 
			Attack3.Analog.Y.getDriveInputSupplier(lstick, 0, 1.0, 1.0),
			Attack3.Analog.X.getDriveInputSupplier(rstick, 0, 1.0, 1.0),
			Attack3.Digital.TRI.getSupplier(lstick),
			Attack3.Digital.TRI.getSupplier(rstick));
		RobotContainer.m_robotDrive.setDefaultCommand(drive_control);
	}
}
