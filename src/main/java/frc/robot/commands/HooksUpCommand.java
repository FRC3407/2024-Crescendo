// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class HooksUpCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  public static final double HOOK_UP_POSITION = 120;
  public static final double HOOK_SPEED = 0.5;
  /** Creates a new HooksUpCommand. */
  public HooksUpCommand(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getHookOnePosition() < HOOK_UP_POSITION)
      m_climberSubsystem.moveHookOne(HOOK_SPEED);
    else
      m_climberSubsystem.moveHookOne(0);

    if (m_climberSubsystem.getHookTwoPosition() < HOOK_UP_POSITION)
      m_climberSubsystem.moveHookTwo(HOOK_SPEED);
    else
      m_climberSubsystem.moveHookTwo(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.moveHookOne(0);
    m_climberSubsystem.moveHookTwo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climberSubsystem.getHookOnePosition() >= HOOK_UP_POSITION &&
            m_climberSubsystem.getHookTwoPosition() >= HOOK_UP_POSITION);
  }
}
