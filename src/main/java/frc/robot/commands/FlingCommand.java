package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.Intake;

public class FlingCommand extends Command {

    private final Flinger m_flinger;
    private final Intake m_intake;
    private boolean notReady;
    private Timer timer;

    /**
     * Spins the flinger
     * 
     * @param flinger
     * @param supplierfling
     * @param supplierIntake
     */
    public FlingCommand(Flinger flinger, Intake intake) {
        this.m_flinger = flinger;
        this.m_intake = intake;
        addRequirements(m_flinger, m_intake);
    }

    @Override
    public void initialize() {
        this.m_intake.intake(0);
        if (this.m_intake.getStarterSenser() && this.m_intake.getStopSenser()) {
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
            notReady = false;
            timer.reset();
            timer.start();
        } else {
            notReady = true;
        }
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)) {
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        this.m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        if (!(this.m_intake.getShooterSenser() || this.m_intake.getStopSenser()) || notReady) {
            return true;
        } else {
            return false;
        }
    }
}