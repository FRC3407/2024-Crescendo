// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.FileSystems;
import java.nio.file.FileSystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.io.IOException;
import java.nio.file.DirectoryStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;
import frc.robot.commands.AutoGoCommand;
import frc.robot.commands.ClimbCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.FlingCommand;
import frc.robot.commands.ManualFlingCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualIntakeCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.controls.ControlSchemeManager;
import frc.robot.controls.Controls;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.commands.HookReleaseCommand;
import frc.robot.commands.ManualFlingCommand;

public class RobotContainer {
  static boolean visionAutoSelection = false;
  private static final ControlSchemeManager controls = new ControlSchemeManager();
  private final Robot robot = new Robot();
  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("Using Wpilib Version " + WPILibVersion.Version);

    NamedCommands.registerCommand("fling_command", new FlingCommand(Controls.m_flinger, Controls.m_intake));
    NamedCommands.registerCommand("intake_command", new IntakeCommand(Controls.m_flinger, Controls.m_intake));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    Controls.setupControls(this.robot, controls, Controls.FeatureLevel.ALLSCHEMES);
  }

  /**
   * Get the selected Auto Command 
   * @return The Auto Command Object
   */
  public Command getAutonomousCommand() {
    if (visionAutoSelection) {
      String visionAuto = NetworkTableInstance.getDefault().getEntry("").getString("null");

      if (visionAuto != "null") {
        return new PathPlannerAuto(visionAuto);
      }
    }

    return autoChooser.getSelected();
  }

  private static double timeOfLastLoop = System.currentTimeMillis();
  /**
   * Polls the control scheme manager to see if the current scheme 
   * should be changed to a more compatible scheme
   */
  public static void loopScheme() {
    // If 2 seconds have elapse since the last control scheme check
    if (System.currentTimeMillis() - timeOfLastLoop > 2000) {
      controls.loopScheme();
      timeOfLastLoop = System.currentTimeMillis();
    }
  }

  // private void ConfigureButtonBindings() {
  //   Joystick l_attack3 = new Joystick(0);
  //   Joystick r_attack3 = new Joystick(1);

  //   GenericHID buttonBox = new GenericHID(2);

  //   JoystickButton button1 = new JoystickButton(buttonBox, 1);
  //   button1.whileTrue(new ClimbCommand(m_climber));

  //   JoystickButton button2 = new JoystickButton(buttonBox, 2);
  //   button2.whileTrue(new HookReleaseCommand(m_climber));

  //   // reverse intake
  //   JoystickButton button3 = new JoystickButton(buttonBox, 3);
  //   button3.whileTrue(new ManualIntakeCommand(m_intake, true));

  //   // manual intake
  //   JoystickButton button4 = new JoystickButton(buttonBox, 4);
  //   button4.whileTrue(new ManualIntakeCommand(m_intake, false));

  //   // reverse fling
  //   JoystickButton button5 = new JoystickButton(buttonBox, 5);
  //   button5.whileTrue(new ManualFlingCommand(m_flinger, true));

  //   // manual fling
  //   JoystickButton button6 = new JoystickButton(buttonBox, 6);
  //   button6.whileTrue(new ManualFlingCommand(m_flinger, false));

  //   JoystickButton button7 = new JoystickButton(buttonBox, 7);
  //   button7.onTrue(new PrintCommand("camera switch (toggle)"));

  //   JoystickButton button8 = new JoystickButton(buttonBox, 8);
  //   button8.onTrue(new PrintCommand("climber switch (toggle)"));

  //   // ---
  //   JoystickButton boostButton = new JoystickButton(l_attack3, 2);
  //   m_driveTrain.setDefaultCommand(
  //       new DriveCommand(m_driveTrain, r_attack3::getX, r_attack3::getY, l_attack3::getX, ()-> boostButton.getAsBoolean()));

  //   JoystickButton flingButton = new JoystickButton(r_attack3, 1);
  //   flingButton.onTrue(new FlingCommand(m_flinger, m_intake));
  //   JoystickButton intakeButton = new JoystickButton(r_attack3, 2);
  //   intakeButton.onTrue(new IntakeCommand(m_flinger, m_intake));
  //   JoystickButton zeroHeadingButton = new JoystickButton(r_attack3, 7);
  //   zeroHeadingButton.onTrue(new ZeroHeadingCommand(m_driveTrain));
  // }
}
