package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;

public class IntakeReverseCommand extends Command {

    private final FloorIntake m_floorIntake;

    /**
     * Runs the intake
     * 
     * @param m_flinger
     * @param m_floorIntake
     */
    public IntakeReverseCommand(FloorIntake m_floorIntake) {
        this.m_floorIntake = m_floorIntake;
        addRequirements(m_floorIntake);
    }

    @Override
    public void initialize() {
        m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED_REVERSE);
    }

    @Override
    public void execute() {
        m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED_REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        m_floorIntake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}