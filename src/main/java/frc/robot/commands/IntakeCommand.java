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
            this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
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