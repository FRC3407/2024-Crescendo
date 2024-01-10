package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeClock extends Command {

    private final Intake m_intake;
    
    /**
     * Spins the shooter clockwise
     * @param shooter
     */
    public IntakeClock(Intake intake) {
        this.m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //this.m_intake.intake(Constants.INTAKE_CLOCK_SPEED);
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