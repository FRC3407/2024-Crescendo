
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorTop;
    private TalonFX shooterMotorBot;

    public Shooter() {
        shooterMotorTop = new TalonFX(Constants.ShooterConstants.kShootTop);
        shooterMotorBot = new TalonFX(Constants.ShooterConstants.kShootBot);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * Spins the shooter, positive speed shoots the ring out
     * 
     * @param speed
     */
    public void shoot(double speed) {
        shooterMotorTop.set(speed);
        shooterMotorBot.set(-speed);
    }
}
