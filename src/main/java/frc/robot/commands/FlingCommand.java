package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class FlingCommand extends Command {

    private final Flinger m_flinger;
    private final FloorIntake m_intake;
    private boolean ringLoaded;
    private Timer timer;
    private Timer postShotTimer;

    /**
     * Spins the flinger
     * 
     * @param flinger
     * @param supplierfling
     * @param supplierIntake
     */
    public FlingCommand(Flinger flinger, FloorIntake intake) {
        this.m_flinger = flinger;
        this.m_intake = intake;
        addRequirements(m_flinger, m_intake);
        timer = new Timer();
        postShotTimer = new Timer();
    }

    @Override
    public void initialize() {
        this.m_intake.intake(0);
        if (this.m_intake.getBotSensor() && this.m_intake.getMidSensor()) {
            // If a ring is in the correct position start backing up the ring
            this.m_intake.intake(-0.1);
            ringLoaded = true;
            timer.reset();
            postShotTimer.reset();
        } else {
            ringLoaded = false;
        }
    }

    @Override
    public void execute() {
        if (!this.m_intake.getMidSensor()&&timer.hasElapsed(0)) {
            //Backs the ring until the mid sensor is false, then starts timer
            timer.start();
            this.m_intake.intake(0);
        }
        if (timer.hasElapsed(0.05)) {
            //Starts the flinger
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
        }
        if (timer.hasElapsed(2)) {
            //Transfers
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
        if(!(this.m_intake.getTopSensor() || this.m_intake.getMidSensor()
        || this.m_intake.getBotSensor()))
        {
            postShotTimer.start();
        }

    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        this.m_intake.intake(0);
        timer.stop();
        postShotTimer.stop();
        timer.reset();
        postShotTimer.reset();
        ringLoaded = false;
    }

    @Override
    public boolean isFinished() {
        // Ends the command if all sensors are false or if a ring isn't loaded
        return (postShotTimer.hasElapsed(0.25) || !ringLoaded);
    }
}