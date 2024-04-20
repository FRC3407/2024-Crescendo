// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public NetworkTableInstance instance;
  public IntegerArrayTopic topic1;
  public final IntegerArraySubscriber idSub;
  // public NetworkTable table;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    instance = NetworkTableInstance.getDefault();
    topic1 = instance.getIntegerArrayTopic("/Vision Server/Pipelines/bv2024/ids");
    idSub = topic1.subscribe(new long[0]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    long[] ids = idSub.get();
    System.out.println("There are " + Integer.toString(ids.length) + " IDs");
    for (long id : ids) {
      System.out.println(id);
    }
  }
}
