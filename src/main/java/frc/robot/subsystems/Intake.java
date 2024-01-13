
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(Constants.IntakeConstants.kIntake);
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
        intakeMotor.set(TalonFXControlMode.Current, speed);
    }
}
