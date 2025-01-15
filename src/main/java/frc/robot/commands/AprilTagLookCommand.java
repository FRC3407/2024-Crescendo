// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagLookCommand extends Command {
  public final VisionSubsystem visionSubsystem;
  public final DriveSubsystem driveSubsystem;
  /** Creates a new AprilTagLookCommand. */
  public AprilTagLookCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem=visionSubsystem;
    this.driveSubsystem=driveSubsystem;
    addRequirements(visionSubsystem,driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = visionSubsystem.camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      if (Math.abs(yaw)<2.0) {
        System.out.println("i did it");
        return;
      }
      DriveCommand command = new DriveCommand(driveSubsystem,
        () -> 0,
        () -> 0,
        () -> Math.copySign(0.2, yaw),
        () -> false
      );
      
      command.schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
