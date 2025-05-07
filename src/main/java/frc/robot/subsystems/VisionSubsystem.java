// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoGoCommand;

public class VisionSubsystem extends SubsystemBase {
  DriveSubsystem m_driveTrain;
  PhotonCamera camera = new PhotonCamera("Back Camera");

  List<PhotonTrackedTarget> lastTargets = List.of();

  List<Integer> codeTargets = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(DriveSubsystem driveSubsystem) {
    System.out.println("I AM VISION!");

    m_driveTrain = driveSubsystem;
  }

  public Command getCommandFromAprilTagID(int id) {
    // Replace this with your actual logic to return a command based on the AprilTag ID
    switch (id) {
      case 3:
        return new AutoGoCommand(m_driveTrain,0.1,0,0); // drive forward
      case 10:
        return new AutoGoCommand(m_driveTrain, 0, 0, 0.1); // turn right
      default:
        return null; // Return null or a default command if the ID is not recognized
    }
  }

  @Override
  public void periodic() {
    // System.out.println("I AM VISIONING RN!");
    PhotonPipelineResult result = camera.getLatestResult();
    for (PhotonTrackedTarget newTarget : result.targets) {
      if (!wasThereAnAprilTag(newTarget.fiducialId)) {
        // wow we just saw a new april tag
        System.out.println("yo i just saw an april tag: "+newTarget.getFiducialId());
        System.out.println(codeTargets);

        // if the target is 35, run the code
        if (newTarget.getFiducialId() == 35) {
          System.out.println("ok im running da code now:");
          System.out.println(codeTargets);

          if (codeTargets.size()>0) {
            SequentialCommandGroup command = new SequentialCommandGroup();
            for (int i=0;i<codeTargets.size();i++) {
              command.addCommands(getCommandFromAprilTagID(codeTargets.get(i)));
            }
            System.out.println(command);
            // for (Command c : command.m_commands)
            command.schedule();
          } // binkus grinkleton moment
          
          codeTargets.clear();

        } else { // if its not 35 add this target to the code
          codeTargets.add(newTarget.getFiducialId());
          System.out.println(codeTargets);
        }
      }
    }
    lastTargets = result.targets;
  }

  boolean wasThereAnAprilTag(int id) {
    for (PhotonTrackedTarget target : lastTargets) {
      if (target.getFiducialId() == id)
        return true;
    }
    return false;
  }
}
