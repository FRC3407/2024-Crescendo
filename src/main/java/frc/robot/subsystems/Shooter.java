
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorTop;
    private TalonFX shooterMotorBot;

    public Shooter() {
        //shooterMotorTop = new TalonFX(Constants.IDTalon.kShootTop);
        //shooterMotorBot = new TalonFX(Constants.IDTalon.kShootBot);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * Spins the shooter, positive is clockwise
     * @param speed
     */
    public void shoot(double speed){
        shooterMotorTop.set(speed);
        shooterMotorBot.set(-speed);
    }
}
