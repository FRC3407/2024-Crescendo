
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flinger extends SubsystemBase {

    private CANSparkMax flingerMotor_1; // here, change names
    private CANSparkMax flingerMotor_2; 
    private double targetSpeed;
    public static boolean flingCommandActive = false;
    
    public Flinger() {
        flingerMotor_1 = new CANSparkMax(Constants.FlingerConstants.flingerCanID_1, MotorType.kBrushless);
        flingerMotor_1.setInverted(true);

        flingerMotor_2 = new CANSparkMax(Constants.FlingerConstants.flingerCanID_2, MotorType.kBrushless);
        flingerMotor_2.setInverted(false);
    }

    @Override
    public void periodic() {
        
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
        flingerMotor_1.set(speed);
        flingerMotor_2.set(speed);
        targetSpeed = speed;
    }

    public double getTargetRPM()
    {
        return targetSpeed*5500;
    }

    public double getRPM_1()
    {
        return flingerMotor_1.getEncoder().getVelocity();
    }

    public double getRPM_2()
    {
        return flingerMotor_2.getEncoder().getVelocity();
    }


    @Override
    public void initSendable(SendableBuilder b) {
        b.addDoubleProperty("Encoder RPM 1", this::getRPM_1, null);
        b.addDoubleProperty("Encoder RPM 2", this::getRPM_2, null);
        b.addDoubleProperty("Target RPM 1", this::getTargetRPM, null);
        b.addDoubleProperty("Target RPM 2", this::getTargetRPM, null);
        b.addDoubleProperty("Output Current 1", this.flingerMotor_1::getOutputCurrent, null);
        b.addDoubleProperty("Output Current 2", this.flingerMotor_2::getOutputCurrent, null);
        b.addDoubleProperty("Bus Voltage 1", this.flingerMotor_1::getBusVoltage, null);
        b.addDoubleProperty("Bus Voltage 2", this.flingerMotor_2::getBusVoltage, null);
    }

}
