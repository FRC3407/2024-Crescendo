// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
  public final DriveSubsystem m_driveSubsystem;
  private final BooleanSupplier m_linearBoostSupplier;
  private final DoubleSupplier m_linearXSupplier;
  private final DoubleSupplier m_linearYSupplier;
  private final DoubleSupplier m_angularSpeedSupplier;

  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier m_linearXSupplier, DoubleSupplier m_linearYSupplier,
      DoubleSupplier m_angularSpeedSupplier, BooleanSupplier m_linearBoostSupplier) {
    this.m_driveSubsystem = subsystem;
    this.m_linearXSupplier = m_linearXSupplier;
    this.m_linearYSupplier = m_linearYSupplier;
    this.m_angularSpeedSupplier = m_angularSpeedSupplier;
    this.m_linearBoostSupplier = m_linearBoostSupplier;

    this.addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double boostValue = m_linearBoostSupplier.getAsBoolean() == true ? 2 : 1;
    double xSpeed = MathUtil.applyDeadband(m_linearYSupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double ySpeed = MathUtil.applyDeadband(m_linearXSupplier.getAsDouble(), OIConstants.kDriveDeadband);
    double rotSpeed = MathUtil.applyDeadband(m_angularSpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);
    ySpeed = ySpeed * boostValue;
    xSpeed = xSpeed * boostValue;
    m_driveSubsystem.drive(
        -xSpeed/2,
        -ySpeed/2,
        -rotSpeed,
        true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
