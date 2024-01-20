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
  private final BooleanSupplier m_linearBoostSupplier2;
  private final DoubleSupplier m_linearXSupplier;
  private final DoubleSupplier m_linearYSupplier;
  private final DoubleSupplier m_angularSpeedSupplier;
  private final BooleanSupplier m_setXSupplier;
  private final BooleanSupplier m_zeroHeadingSupplier;

  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier m_linearXSupplier, DoubleSupplier m_linearYSupplier,
      DoubleSupplier m_angularSpeedSupplier, BooleanSupplier m_linearBoostSupplier,
      BooleanSupplier m_linearBoostSupplier2, BooleanSupplier m_setXSupplier, BooleanSupplier m_zeroHeadingSupplier) {
    this.m_driveSubsystem = subsystem;
    this.m_linearXSupplier = m_linearXSupplier;
    this.m_linearYSupplier = m_linearYSupplier;
    this.m_angularSpeedSupplier = m_angularSpeedSupplier;
    this.m_linearBoostSupplier2 = m_linearBoostSupplier2;
    this.m_linearBoostSupplier = m_linearBoostSupplier;
    this.m_setXSupplier = m_setXSupplier;
    this.m_zeroHeadingSupplier = m_zeroHeadingSupplier;
    this.addRequirements(m_driveSubsystem);
  }

  public DriveCommand(DriveSubsystem subsystem, DoubleSupplier m_linearXSupplier, DoubleSupplier m_linearYSupplier,
      DoubleSupplier m_angularSpeedSupplier, DoubleSupplier m_linearBoostSupplier,
      DoubleSupplier m_linearBoostSupplier2, BooleanSupplier m_setXSupplier, BooleanSupplier m_zeroHeadingSupplier) {
    this.m_driveSubsystem = subsystem;
    this.m_linearXSupplier = m_linearXSupplier;
    this.m_linearYSupplier = m_linearYSupplier;
    this.m_angularSpeedSupplier = m_angularSpeedSupplier;
    this.m_linearBoostSupplier2 = () -> m_linearBoostSupplier2.getAsDouble() >= 0.5;
    this.m_linearBoostSupplier = () -> m_linearBoostSupplier.getAsDouble() >= 0.5;
    this.m_setXSupplier = m_setXSupplier;
    this.m_zeroHeadingSupplier = m_zeroHeadingSupplier;
    this.addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(
        -MathUtil.applyDeadband(
            m_linearYSupplier.getAsDouble() / 2.25 * (m_linearBoostSupplier.getAsBoolean() == true ? 1.5 : 1)
                * (m_linearBoostSupplier2.getAsBoolean() == true ? 1.5 : 1)
                * (m_linearBoostSupplier.getAsBoolean() == true ? 1.5 : 1),
            OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(
            m_linearXSupplier.getAsDouble() / 2.25 * (m_linearBoostSupplier.getAsBoolean() == true ? 1.5 : 1)
                * (m_linearBoostSupplier2.getAsBoolean() == true ? 1.5 : 1)
                * (m_linearBoostSupplier.getAsBoolean() == true ? 1.5 : 1),
            OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_angularSpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband),
        true, true);
    if (m_setXSupplier.getAsBoolean()) {
      m_driveSubsystem.setX();
    }
    if (m_zeroHeadingSupplier.getAsBoolean()) {
      m_driveSubsystem.zeroHeading();
    }
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
