
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor;
    private DigitalInput senseStart = new DigitalInput(0);
    private DigitalInput senseStop = new DigitalInput(1);
    private DigitalInput senseShoot = new DigitalInput(2);

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.motorCanID, MotorType.kBrushless);
        

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * Spins the intake, positive is clockwise
     * 
     * @param speed
     */
    public void intake(double speed) {
        intakeMotor.set(speed);
    }

    // Gets the value of the start senser
    public boolean getStarterSenser() {
        return senseStart.get();
    }
    // gets the value of the middle senser
    public boolean getStopSenser() {
        return senseStop.get();
    }
    // gets the value of the shooter senser
    public boolean getShooterSenser() {
        return senseStop.get();
    }
}
