
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntake extends SubsystemBase {

    private CANSparkMax intakeMotor;
    private DigitalInput sensorBot = new DigitalInput(1);
    private DigitalInput sensorMid = new DigitalInput(2);
    private DigitalInput sensorTop = new DigitalInput(3);

    public FloorIntake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.motorCanID, MotorType.kBrushless);
        intakeMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bot Sensor", getBotSensor());
        SmartDashboard.putBoolean("Top Sensor", getTopSensor());
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

    /**
     * Gets the value of the bottom sensir
     * 
     * @return The value of the bottom sensor
     */
    public boolean getBotSensor() {
        return sensorBot.get();
    }

    /**
     * Gets the value of the top sensir
     * 
     * @return The value of the top sensor
     */
    public boolean getTopSensor() {
        return sensorTop.get();
    }
}
