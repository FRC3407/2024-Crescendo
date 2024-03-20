package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class FlingCommand extends Command {

    private final Flinger m_flinger;
    private final FloorIntake m_intake;
    private Timer timer = new Timer();

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
    }

    @Override
    public void initialize() {
        timer.reset();
     }

    @Override
    public void execute() {
        this.m_intake.intake(Constants.IntakeConstants.INTAKE_SPEED);
        this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
        
        if (!m_intake.getBotSensor() && !m_intake.getTopSensor()){
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_flinger.fling(0);
        this.m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        // Ends the command if all sensors are false
        return timer.hasElapsed(2.5);
    }

}