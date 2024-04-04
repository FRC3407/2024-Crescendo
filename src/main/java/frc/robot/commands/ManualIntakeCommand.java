package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;

public class ManualIntakeCommand extends Command {

    private final FloorIntake m_floorIntake;
    private boolean reversed;

    /**
     * Runs the intake
     * 
     * @param m_flinger
     * @param m_floorIntake
     */
    public ManualIntakeCommand(FloorIntake m_floorIntake, boolean reversed) {
        this.m_floorIntake = m_floorIntake;
        this.reversed = reversed;
        addRequirements(m_floorIntake);
    }

    @Override
    public void initialize() {
        // ---
    }

    @Override
    public void execute() {
        if (reversed) {
            m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED_REVERSE);
        } else {
            m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        }
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