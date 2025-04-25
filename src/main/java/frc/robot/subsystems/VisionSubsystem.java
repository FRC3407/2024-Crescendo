// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Back Camera");

  List<PhotonTrackedTarget> lastTargets = List.of();

  List<Integer> codeTargets = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    System.out.println("I AM VISION!");
  }

  @Override
  public void periodic() {
    // System.out.println("I AM VISIONING RN!");
    PhotonPipelineResult result = camera.getLatestResult();
    for (PhotonTrackedTarget newTarget : result.targets) {
      if (!wasThereAnAprilTag(newTarget.fiducialId)) {
        // wow we just saw a new april tag
        System.out.println("yo i just saw an april tag: "+newTarget.getFiducialId());
        if (newTarget.getFiducialId() == 35) {
          System.out.println("ok im running da code now:");
          System.out.println(codeTargets);
          
          while (!codeTargets.isEmpty())
            codeTargets.remove(0);
        } else {
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
