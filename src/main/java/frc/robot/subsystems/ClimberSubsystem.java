// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotorOne;
  private CANSparkMax climberMoterTwo; 

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  climberMotorOne = new CANSparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
  climberMoterTwo = new CANSparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void climb(double speed){
    climberMotorOne.set(speed);
    climberMoterTwo.set(speed); 
  }

}
