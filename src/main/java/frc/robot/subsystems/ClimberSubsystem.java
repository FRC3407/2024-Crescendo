// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMaxPIDController;
public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotor;
  private SparkMaxPIDController climberPIDController;
  private RelativeEncoder climberEncoder;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  climberMotor = new CANSparkMax(Constants.ClimberConstants.climberCanID, MotorType.kBrushless);
  climberPIDController = climberMotor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void climb(double speed){
    climberMotor.set(speed);
  }
  public void hold(currentSpeed){
    climberPIDController.setReference()
    
  }
  public double getRPM(){
    return climberMotor.getEncoder().getVelocity();
    }

}
