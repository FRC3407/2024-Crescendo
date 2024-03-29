package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;

public class ManualFlingCommand extends Command {

    private final Flinger m_flinger;
    private boolean reversed;

    /**
     * Positions the ring, spins the flinger to max speed, and then fires
     * 
     * @param m_flinger
     */
    public ManualFlingCommand(Flinger m_flinger, boolean reversed) {
        this.m_flinger = m_flinger;
        this.reversed = reversed;
        addRequirements(m_flinger);
    }

    @Override
    public void initialize() {
        Flinger.flingCommandActive = true;
     }

    @Override
    public void execute() {
        if (reversed) {
            m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED);
        } else {
            m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        Flinger.flingCommandActive = false;
    }

    @Override
    public boolean isFinished() {
        // Ends the command if the post shot timer has surpassed half a second
        return false; // ????
    }

}