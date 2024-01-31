package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Shooter m_shooter;
    private final BooleanSupplier supplierShoot;
    private final BooleanSupplier supplierIntake;

    /**
     * Spins the shooter
     * 
     * @param shooter
     * @param supplierShoot
     * @param supplierIntake
     */
    public ShootCommand(Shooter shooter, DoubleSupplier supplierShoot, DoubleSupplier supplierIntake) {
        this.m_shooter = shooter;
        this.supplierShoot = () -> (supplierShoot.getAsDouble() >= 0.2);
        this.supplierIntake = () -> (supplierIntake.getAsDouble() >= 0.2);
        addRequirements(m_shooter);
    }

    /**
     * Spins the shooter
     * 
     * @param shooter
     * @param supplierShoot
     * @param supplierIntake
     */
    public ShootCommand(Shooter shooter, BooleanSupplier supplierShoot, BooleanSupplier supplierIntake) {
        this.m_shooter = shooter;
        this.supplierShoot = supplierShoot;
        this.supplierIntake = supplierIntake;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (supplierShoot.getAsBoolean() && !supplierIntake.getAsBoolean()) {
            this.m_shooter.shoot(Constants.ShooterConstants.SHOOTER_SHOOT_SPEED);
        } else if (!supplierShoot.getAsBoolean() && supplierIntake.getAsBoolean()) {
            this.m_shooter.shoot(Constants.ShooterConstants.SHOOTER_INTAKE_SPEED);
        } else {
            this.m_shooter.shoot(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_shooter.shoot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}