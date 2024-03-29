// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;
import frc.robot.commands.AutoGoCommand;
import frc.robot.commands.DriveCommand;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.commands.FlingCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ZeroHeadingCommand;

public class RobotContainer {
  DriveSubsystem m_driveTrain = new DriveSubsystem();
  Flinger m_flinger = new Flinger();
  FloorIntake m_intake = new FloorIntake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("Using Wpilib Version " + WPILibVersion.Version);
    ConfigureButtonBindings();

    NamedCommands.registerCommand("fling_command", new FlingCommand(m_flinger, m_intake));
    NamedCommands.registerCommand("intake_command", new IntakeCommand(m_flinger, m_intake));
  }

  public Command getAutonomousCommand() {
    // return new AutoGoCommand(m_driveTrain);
    return new PathPlannerAuto("test_auto");
  }

  private void ConfigureButtonBindings() {
    Joystick l_attack3 = new Joystick(0);
    Joystick r_attack3 = new Joystick(1);
    JoystickButton boostButton = new JoystickButton(l_attack3, 2);
    m_driveTrain.setDefaultCommand(
        new DriveCommand(m_driveTrain, r_attack3::getX, r_attack3::getY, l_attack3::getX, ()-> boostButton.getAsBoolean()));

    JoystickButton flingButton = new JoystickButton(r_attack3, 1);
    flingButton.onTrue(new FlingCommand(m_flinger, m_intake));
    JoystickButton intakeButton = new JoystickButton(r_attack3, 2);
    intakeButton.onTrue(new IntakeCommand(m_flinger, m_intake));
    JoystickButton zeroHeadingButton = new JoystickButton(r_attack3, 7);
    zeroHeadingButton.onTrue(new ZeroHeadingCommand(m_driveTrain));
  }
  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  // public Command getAutonomousCommand() {
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // exampleTrajectory,
  // Controls.m_driveTrain::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // Controls.m_driveTrain::setModuleStates,
  // Controls.m_driveTrain);

  // // Reset odometry to the starting pose of the trajectory.
  // Controls.m_driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> Controls.m_driveTrain.drive(0,
  // 0, 0, false, false));
  // }
}
