// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ClimberSubsystem extends SubsystemBase {

  private SparkMax climberMotorOne;
  private SparkMax climberMotorTwo; 
  private SparkMax hookRelease; 

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  climberMotorOne = new SparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
  climberMotorOne.setInverted(true); 

  climberMotorTwo = new SparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
  climberMotorTwo.setInverted(false); 

  hookRelease = new SparkMax(Constants.ClimberConstants.hookReleaseCanID, MotorType.kBrushed); 
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

}
