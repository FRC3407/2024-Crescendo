
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private WPI_TalonFX shooterMotorTop;
    private WPI_TalonFX shooterMotorBot;

    public Shooter() {
        //shooterMotorTop = new WPI_TalonFX(Constants.IDTalon.kShootTop);
        //shooterMotorBot = new WPI_TalonFX(Constants.IDTalon.kShootBot);
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
