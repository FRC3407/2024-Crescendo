
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flinger extends SubsystemBase {

    private CANSparkMax flingerMotor;
    private double targetSpeed;

    public Flinger() {
        flingerMotor = new CANSparkMax(Constants.FlingerConstants.flingerCanID, MotorType.kBrushless);
        flingerMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flinger Encoder Velocity", getRPM());
        SmartDashboard.putNumber("Flinger Target Velocity", getTargetRPM());
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * Spins the flinger, positive speed shoots the ring out
     * 
     * @param speed
     */
    public void fling(double speed) {
        flingerMotor.set(speed);
        targetSpeed = speed;
    }

    public double getTargetRPM()
    {
        return targetSpeed*5500;
    }

    public double getRPM(){
        return flingerMotor.getEncoder().getVelocity();
    }

}
