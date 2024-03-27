package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Flinger;
import frc.robot.subsystems.FloorIntake;

public class IntakeCommand extends Command {

    private final FloorIntake m_floorIntake;
    private final Flinger m_flinger;

    /**
     * Runs the intake
     * 
     * @param m_flinger
     * @param m_floorIntake
     */
    public IntakeCommand(Flinger m_flinger, FloorIntake m_floorIntake) {
        this.m_floorIntake = m_floorIntake;
        this.m_flinger = m_flinger;
        addRequirements(m_floorIntake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        boolean isTop = m_floorIntake.getTopSensor();
        boolean isBot = m_floorIntake.getBotSensor();

        if(!isBot && !isTop){
            this.m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED);
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED); 
        }
        if (isBot && !isTop){
            this.m_floorIntake.intake(Constants.IntakeConstants.INTAKE_ADJUST_SPEED);
        }
        if(isTop && !isBot){
            this.m_floorIntake.intake(-Constants.IntakeConstants.INTAKE_ADJUST_SPEED);
            // this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED); 
            // come back, make constant for negative intake speed
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_floorIntake.intake(0);
        this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED);
    }

    @Override
    public boolean isFinished() {
        return (this.m_floorIntake.getBotSensor() && this.m_floorIntake.getTopSensor()) || Flinger.flingCommandActive;
    }
}