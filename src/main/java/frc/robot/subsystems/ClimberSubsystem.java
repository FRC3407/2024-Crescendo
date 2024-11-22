// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotorOne;
  private RelativeEncoder climberEncoderOne;
  private CANSparkMax climberMotorTwo; 
  private RelativeEncoder climberEncoderTwo;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotorOne = new CANSparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
    climberMotorOne.setInverted(true); 
    climberEncoderOne = climberMotorOne.getEncoder();
    climberEncoderOne.setPositionConversionFactor(360/42);

    climberMotorTwo = new CANSparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
    climberMotorTwo.setInverted(false); 
    climberEncoderTwo = climberMotorTwo.getEncoder();
    climberEncoderTwo.setPositionConversionFactor(360/42);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveHookOne(double speed) {
    climberMotorOne.set(speed);
  }
  public void moveHookTwo(double speed) {
    climberMotorTwo.set(speed);
  }
  public double getHookOnePosition() {
    return climberEncoderOne.getPosition();
  }
  public double getHookTwoPosition() {
    return climberEncoderTwo.getPosition();
  }

}
