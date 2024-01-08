// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  public final DriveSubsystem m_driveSubsystem;
  private final BooleanSupplier m_speedBoostSupplier;
  private final BooleanSupplier m_angularSpeedBoostSupplier;
  private final DoubleSupplier m_linearXSupplier;
  private final DoubleSupplier m_linearYSupplier;
  private final DoubleSupplier m_angularSpeedSupplier;

  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier m_linearXSupplier, DoubleSupplier m_linearYSupplier, DoubleSupplier m_angularSpeedSupplier, BooleanSupplier m_linearBoostSupplier, BooleanSupplier m_angularBoostSupplier) {
    this.m_driveSubsystem = subsystem;
    this.m_linearXSupplier = m_linearXSupplier;
    this.m_linearYSupplier = m_linearYSupplier;
    this.m_angularSpeedSupplier = m_angularSpeedSupplier;
    this.m_angularSpeedBoostSupplier = m_angularBoostSupplier;
    this.m_speedBoostSupplier = m_linearBoostSupplier;
    this.addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(
      -MathUtil.applyDeadband(m_linearYSupplier.getAsDouble()/2.25*(m_speedBoostSupplier.getAsBoolean()==true?1.5:1)
      *(m_speedBoostSupplier.getAsBoolean()==true?1.5:1), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_linearXSupplier.getAsDouble()/2.25*(m_speedBoostSupplier.getAsBoolean()==true?1.5:1)
      *(m_speedBoostSupplier.getAsBoolean()==true?1.5:1), OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_angularSpeedSupplier.getAsDouble()/2.25*(m_angularSpeedBoostSupplier.getAsBoolean()==true?1.5:1)
      *(m_angularSpeedBoostSupplier.getAsBoolean()==true?1.5:1), OIConstants.kDriveDeadband),
      true, true);
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
