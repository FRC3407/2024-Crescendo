
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntake extends SubsystemBase {

    private CANSparkMax intakeMotor;
    private DigitalInput sensorBot = new DigitalInput(Constants.IntakeConstants.BOT_DIO_SENSOR);
    private DigitalInput sensorTop = new DigitalInput(Constants.IntakeConstants.TOP_DIO_SENSOR);

    public FloorIntake() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.motorCanID, MotorType.kBrushless);
        intakeMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bot Sensor", getBotSensor());
        SmartDashboard.putBoolean("Top Sensor", getTopSensor());
        SmartDashboard.putNumber("Intake Velocity", intakeMotor.getEncoder().getVelocity());
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
     * Gets the value of the bottom sensor
     * 
     * @return The value of the bottom sensor
     */
    public boolean getBotSensor() {
        return sensorBot.get();
    }

    /**
     * Gets the value of the top sensor
     * 
     * @return The value of the top sensor
     */
    public boolean getTopSensor() {
        return sensorTop.get();
    }
}
