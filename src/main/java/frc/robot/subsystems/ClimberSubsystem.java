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
  public void climb(double speed){
    climberMotorOne.set(speed);
    climberMotorTwo.set(speed); 
  }
  public void release(double speed){
    hookRelease.set(speed); 
  }

  @Override
  public void initSendable(SendableBuilder b) {
    b.addDoubleProperty("Climber 1 encoder position", ()->this.climberMotorOne.getEncoder().getPosition(), null);
    b.addDoubleProperty("Climber 2 encoder position", ()->this.climberMotorTwo.getEncoder().getPosition(), null);
    b.addDoubleProperty("Climber hook release encoder position", ()->this.hookRelease.getEncoder().getPosition(), null);
    b.addDoubleProperty("Climber 1 current", ()->this.climberMotorOne.getOutputCurrent(), null);
    b.addDoubleProperty("Climber 2 current", ()->this.climberMotorTwo.getOutputCurrent(), null);
    b.addDoubleProperty("Climber hook release current", ()->this.hookRelease.getOutputCurrent(), null);
    b.addDoubleProperty("Climber 1 raw output", ()->this.climberMotorOne.getAppliedOutput(), null);
    b.addDoubleProperty("Climber 2 raw output", ()->this.climberMotorTwo.getAppliedOutput(), null);
    b.addDoubleProperty("Climber hook release raw output", ()->this.hookRelease.getAppliedOutput(), null);
    b.addDoubleProperty("Climber 1 voltage", ()->this.climberMotorOne.getBusVoltage(), null);
    b.addDoubleProperty("Climber 2 voltage", ()->this.climberMotorTwo.getBusVoltage(), null);
    b.addDoubleProperty("Climber hook release voltage", ()->this.hookRelease.getBusVoltage(), null);
  }

}
