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
     * @param shooter
     */
    public IntakeCommand(Flinger flinger, FloorIntake floorIntake) {
        this.m_floorIntake = floorIntake;
        this.m_flinger = flinger;
        addRequirements(m_floorIntake);
    }

    // come back, should this be || or &&? || would work for an && situation
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
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED); 
            // come back, make constant for negative intake speed
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_floorIntake.intake(0);
        this.m_flinger.fling(Constants.FlingerConstants.FLINGER_SHOOT_SPEED*0.50);
    }

    @Override
    public boolean isFinished() {
        return this.m_floorIntake.getBotSensor() && this.m_floorIntake.getTopSensor();
    }
}