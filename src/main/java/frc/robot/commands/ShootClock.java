package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootClock extends CommandBase {

    private final Shooter m_shoot;
    
    /**
     * Spins the shooter clockwise
     * @param shooter
     */
    public ShootClock(Shooter shooter) {
        this.m_shoot = shooter;
        addRequirements(m_shoot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //this.m_shooter.shoot(Constants.SHOOTER_CLOCK_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_shoot.shoot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}