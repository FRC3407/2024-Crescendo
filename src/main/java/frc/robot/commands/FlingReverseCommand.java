package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class FlingReverseCommand extends Command {

    private final Flinger m_flinger;
    private Timer postShotTimer = new Timer();

    /**
     * Positions the ring, spins the flinger to max speed, and then fires
     * 
     * @param m_flinger
     */
    public FlingReverseCommand(Flinger m_flinger) {
        this.m_flinger = m_flinger;
        addRequirements(m_flinger);
    }

    @Override
    public void initialize() {
        postShotTimer.reset();
        Flinger.flingCommandActive = true;
     }

    @Override
    public void execute() {
        this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        Flinger.flingCommandActive = false;
    }

    @Override
    public boolean isFinished() {
        // Ends the command if the post shot timer has surpassed half a second
        return false;
    }

}