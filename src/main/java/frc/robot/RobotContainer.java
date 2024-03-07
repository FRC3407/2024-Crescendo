// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.controls.ControlSchemeManager;
import frc.robot.controls.Controls;

public class RobotContainer extends TimedRobot {

  private static final ControlSchemeManager controls = new ControlSchemeManager();
  private final Robot robot = new Robot();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("Using Wpilib Version " + WPILibVersion.Version);
    Controls.setupControls(this.robot, controls, Controls.FeatureLevel.ALLSCHEMES);
  }
  
  private static double timeSinceLastLoop = System.currentTimeMillis();

  /**
   * Polls the control scheme manager to see if the current scheme 
   * should be changed to a more compatible scheme
   */
  public static void loopScheme() {
    // If 2 seconds have elapse since the last control scheme check
    if (System.currentTimeMillis() - timeSinceLastLoop > 2000) {
      controls.loopScheme();
      timeSinceLastLoop = System.currentTimeMillis();
    }
  }
}
