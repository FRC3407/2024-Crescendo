package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class FlingCommand extends Command {

    private final Flinger m_flinger;
    private final FloorIntake m_intake;
    private Timer postShotTimer = new Timer();

    /**
     * Positions the ring, spins the flinger to max speed, and then fires
     * 
     * @param m_flinger
     * @param m_intake
     */
    public FlingCommand(Flinger m_flinger, FloorIntake m_intake) {
        this.m_flinger = m_flinger;
        this.m_intake = m_intake;
        addRequirements(m_flinger, m_intake);
    }

    @Override
    public void initialize() {
        postShotTimer.reset();
        Flinger.flingCommandActive = true;
     }

    @Override
    public void execute() {
        this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
        
        if (!m_intake.getBotSensor() && !m_intake.getTopSensor()){
            postShotTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        this.m_intake.intake(0);
        Flinger.flingCommandActive = false;
    }

    @Override
    public boolean isFinished() {
        // Ends the command if the post shot timer has surpassed half a second
        return postShotTimer.hasElapsed(.5);
    }

}