package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class FlingCommand extends Command {

    private final Flinger m_flinger;
    private final FloorIntake m_intake;
    private boolean ringLoaded;
    private Timer postShotTimer;
    private boolean startFlinger = false;
    private ArrayList<Double> rpmList = new ArrayList<Double>();

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
        postShotTimer = new Timer();
    }

    @Override
    public void initialize() {
        this.m_intake.intake(0);
        if (this.m_intake.getMidSensor()) {
            // If a ring is in the correct position start backing up the ring
            this.m_intake.intake(-0.1);
            ringLoaded = true;
            startFlinger = false;
            postShotTimer.reset();
            rpmList.clear();
            Flinger.flingCommandActive = true;
        } else {
            ringLoaded = false;
        }
    }

    @Override
    public void execute() {
        if (!this.m_intake.getMidSensor() && !startFlinger) {
            // Backs the ring until the mid sensor is false, then starts timer
            startFlinger = true;
            this.m_intake.intake(0);
        }
        if (startFlinger) {
            // Starts the flinger
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
        }
        if (readyToFire() && startFlinger) {
            // Transfers
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
        if (!(this.m_intake.getTopSensor() || this.m_intake.getMidSensor()
                || this.m_intake.getBotSensor())) {
            postShotTimer.start();
        }

    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        this.m_intake.intake(0);
        postShotTimer.stop();
        postShotTimer.reset();
        ringLoaded = false;
        rpmList.clear();
        Flinger.flingCommandActive = false;
    }

    @Override
    public boolean isFinished() {
        // Ends the command if all sensors are false or if a ring isn't loaded
        return (postShotTimer.hasElapsed(0.25) || !ringLoaded);
    }

    /**
     * @return True if the flinger if the rpm of the flinger stable and running
     *         for 10 consecutive ticks, indicating its at its max speed
     */
    public boolean readyToFire() {
        double currentRPM = m_flinger.getRPM();
        rpmList.add(0, currentRPM);
        if (rpmList.size() != 10) {
            return false;
        }
        if (rpmList.size() == 10) {
            rpmList.remove(9);
        }
        for (Double rpmValue : rpmList) {
            if (Math.abs(rpmValue.doubleValue() - currentRPM) >= 60 || rpmValue < 200) {
                return false;
            }
        }
        return true;
    }
}