// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class FallCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  public static final double HOOK_SPEED = -0.2;
  private double hookspeed = HOOK_SPEED;
  private boolean hookside;
  /** Creates a new ClimbCommand. */
  public FallCommand(ClimberSubsystem climberSubsystem, double speed, boolean side) { //left is true, right is false side
    m_climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
    hookspeed = -speed;
    hookside = side;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hookside == true){
      m_climberSubsystem.moveHookOne(hookspeed);
      
    }
    else {
      m_climberSubsystem.moveHookTwo(hookspeed);
    }
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
    return false;
  }
}