// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  UsbCamera camera1;
  UsbCamera camera2;
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setResolution(320, 240);
    camera1.setFPS(7);

    camera2 = CameraServer.startAutomaticCapture(1);
    camera1.setResolution(320, 240);
    camera1.setFPS(7);
    // you left your computer unattended lol 
    // dont push to main!
    // you are a wonderful programmer ;)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
