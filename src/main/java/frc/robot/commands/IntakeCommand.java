package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
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

    @Override
    public void initialize() {
        if (!this.m_floorIntake.getMidSensor()) {
            // If there isn't a ring loaded, start intake
            this.m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED);
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED);
        }
    }

    @Override
    public void execute() {
        if (this.m_floorIntake.getTopSensor() && !this.m_floorIntake.getMidSensor() && !this.m_floorIntake.getBotSensor()) {
            this.m_floorIntake.intake(0);
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED);
        }
        if (this.m_floorIntake.getTopSensor() && this.m_floorIntake.getMidSensor() && !this.m_floorIntake.getBotSensor()) {
            this.m_floorIntake.intake(0);
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED*2);
        } else if (this.m_floorIntake.getMidSensor() && !this.m_floorIntake.getBotSensor()) {
            this.m_flinger.fling(0);
            this.m_floorIntake.intake(-Constants.IntakeConstants.INTAKE_SPEED * 0.1);
        } else if (this.m_floorIntake.getBotSensor() && !this.m_floorIntake.getMidSensor()
                && !this.m_floorIntake.getTopSensor()) {
            this.m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED);
            this.m_flinger.fling(0);
        } else {
            this.m_floorIntake.intake(Constants.IntakeConstants.INTAKE_SPEED);
            this.m_flinger.fling(Constants.FlingerConstants.FLINGER_INTAKE_SPEED);
        }

    }

    @Override
    public void end(boolean interrupted) {
        this.m_floorIntake.intake(0);
        this.m_flinger.fling(0);
    }

    @Override
    public boolean isFinished() {
        if (this.m_floorIntake.getMidSensor() && this.m_floorIntake.getBotSensor()) {
            return true;
        } else {
            return false;
        }
    }
}