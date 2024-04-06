// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotorOne;
  private CANSparkMax climberMotorTwo; 
  private CANSparkMax hookRelease;
  
  private double
    m_climberSpeedTarget = 0.0,
    m_hookReleaseSpeedTarget = 0.0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotorOne = new CANSparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
    climberMotorOne.setInverted(true); 

    climberMotorTwo = new CANSparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
    climberMotorTwo.setInverted(false); 

    hookRelease = new CANSparkMax(Constants.ClimberConstants.hookReleaseCanID, MotorType.kBrushed); 
    hookRelease.setInverted(false); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void climb(double speed) {
    m_climberSpeedTarget = speed;
    climberMotorOne.set(speed);
    climberMotorTwo.set(speed); 
  }
  public void release(double speed) {
    m_hookReleaseSpeedTarget = speed;
    hookRelease.set(speed); 
  }

  @Override
  public void initSendable(SendableBuilder b) {
    // motors
    b.addDoubleProperty("Climber Motor 1/Encoder Position", ()->this.climberMotorOne.getEncoder().getPosition(), null);
    b.addDoubleProperty("Climber Motor 2/Encoder Position", ()->this.climberMotorTwo.getEncoder().getPosition(), null);
    // b.addDoubleProperty("Climber Hook Motor/Encoder Position", ()->this.hookRelease.getEncoder().getPosition(), null);   // DO NOT UNCOMMENT -- THIS CAUSES A CRASH!
    b.addDoubleProperty("Climber Motor 1/Current", ()->this.climberMotorOne.getOutputCurrent(), null);
    b.addDoubleProperty("Climber Motor 2/Current", ()->this.climberMotorTwo.getOutputCurrent(), null);
    b.addDoubleProperty("Climber Hook Motor/Current", ()->this.hookRelease.getOutputCurrent(), null);
    b.addDoubleProperty("Climber Motor 1/Bus Voltage", ()->this.climberMotorOne.getBusVoltage(), null);
    b.addDoubleProperty("Climber Motor 2/Bus Voltage", ()->this.climberMotorTwo.getBusVoltage(), null);
    b.addDoubleProperty("Climber Hook Motor/Bus Voltage", ()->this.hookRelease.getBusVoltage(), null);
    b.addDoubleProperty("Climber Motor 1/Applied Speed", ()->this.climberMotorOne.get(), null);
    b.addDoubleProperty("Climber Motor 2/Applied Speed", ()->this.climberMotorTwo.get(), null);
    b.addDoubleProperty("Climber Hook Motor/Applied Speed", ()->this.hookRelease.get(), null);
    b.addDoubleProperty("Climber Motor 1/Temperature", ()->this.climberMotorOne.getMotorTemperature(), null);
    b.addDoubleProperty("Climber Motor 2/Temperature", ()->this.climberMotorTwo.getMotorTemperature(), null);
    b.addDoubleProperty("Climber Hook Motor/Temperature", ()->this.hookRelease.getMotorTemperature(), null);
    // targets
    b.addDoubleProperty("Climber Speed Target", ()->this.m_climberSpeedTarget, null);
    b.addDoubleProperty("Climber Hook Release Speed Target", ()->this.m_hookReleaseSpeedTarget, null);
  }

}
