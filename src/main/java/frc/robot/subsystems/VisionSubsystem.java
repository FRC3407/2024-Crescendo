// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public NetworkTableInstance instance;
  public IntegerArrayTopic topic1;
  public final IntegerArraySubscriber idSub;
  private boolean startAdd;
  private ArrayList<Long> idList = new ArrayList<Long>();

  // public NetworkTable table;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    instance = NetworkTableInstance.getDefault();
    topic1 = instance.getIntegerArrayTopic("/Vision Server/Pipelines/driverCam/ids");
    idSub = topic1.subscribe(new long[0]);
    startAdd = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    long[] ids = idSub.get();
    // if (ids.length != 0)
  }

  // Search for a tag ID in the tags list
  public boolean isTagVisible(int tagID) {
    long[] ids = idSub.get();
    if (ids.length == 0) return false;
    if (ids.length == 1 && ids[0] == tagID) return true;
    for (int i=0;i<ids.length;i++) {
      if (ids[i]==tagID)
        return true;
    }
    return false;
  }

  public long whichTagVisible() {
    long[] ids = idSub.get();
    if (ids.length == 0)
    {
      return -1;
    }
    return ids[0];
  }
  
  public void getTagList() {
    if(isTagVisible(5)) {
      startAdd = true;
      boolean onContinue = false;
    }
    if (isTagVisible(35)) {
      startAdd = false;
    }
    if(startAdd && !isTagVisible(35) && !isTagVisible(5) && !isTagVisible(11)) {
      long id = whichTagVisible();
      idList.add(id);
    }



  }

  // public void onTagVisible(int tagID, Command cmd) { }

}
