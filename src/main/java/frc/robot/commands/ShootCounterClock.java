package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShootCounterClock extends Command {

  private final Shooter m_shooter;

  /**
   * Spins the shooter counter-clockwise
   * @param shooter
   */
  public ShootCounterClock(Shooter shooter) {
    this.m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //this.m_shooter.shoot(Constants.SHOOTER_COUNTERCLOCK_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    this.m_shooter.shoot(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
