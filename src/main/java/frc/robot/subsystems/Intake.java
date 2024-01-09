
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private WPI_TalonFX intakeMotor;

    public Intake() {
        //intakeMotor = new WPI_TalonFX(Constants.IDTalon.kIntake);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * Spins the intake, positive is clockwise
     * @param speed
     */
    public void intake(double speed){
        intakeMotor.set(speed);
    }
}
