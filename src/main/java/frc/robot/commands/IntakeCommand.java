package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake m_intake;
    private BooleanSupplier intakeSupplier;
    private int selector = 0;

    /**
     * Runs the intake
     * 
     * @param shooter
     */
    public IntakeCommand(Intake intake, DoubleSupplier intakeSupplier) {
        this.m_intake = intake;
        this.intakeSupplier = () -> (intakeSupplier.getAsDouble() >= 0.2);
        addRequirements(m_intake);
    }

    /**
     * Runs the intake
     * 
     * @param shooter
     */
    public IntakeCommand(Intake intake, BooleanSupplier intakeSupplier) {
        this.m_intake = intake;
        this.intakeSupplier = intakeSupplier;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intakeSupplier.getAsBoolean()) {
            if (selector == 0) {
                selector = 1;
            }
        }
        if (selector == 1) {
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
        if (this.m_intake.getStopSenser()) {
            if (this.m_intake.getStarterSenser()) {
            this.m_intake.intake(0);
            selector = 3;
            }
            else {
                this.m_intake.intake(-Constants.IntakeConstants.INTAKE_SPEED);
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}