package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake m_intake;

    /**
     * Runs the intake
     * 
     * @param shooter
     */
    public IntakeCommand(Intake intake) {
        this.m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        if (!this.m_intake.getStopSenser()) {
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
    }

    @Override
    public void execute() {
        if (this.m_intake.getStopSenser() && !this.m_intake.getStarterSenser()) {
            this.m_intake.intake(-Constants.IntakeConstants.INTAKE_SPEED * 0.5);
        } else {
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);

        }

    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        if (this.m_intake.getStopSenser() && this.m_intake.getStarterSenser()) {
            return true;
        } else {
            return false;
        }
    }
}