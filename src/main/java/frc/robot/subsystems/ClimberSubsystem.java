// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotorOne;
  private SparkMaxPIDController pidControllerOne;
  private CANSparkMax climberMotorTwo;
  private SparkMaxPIDController pidControllerTwo;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotorOne = new CANSparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
    climberMotorOne.setInverted(true);
    pidControllerOne = climberMotorOne.getPIDController();
    pidControllerOne.setP(1);    // TODO: tune PID parameters
    pidControllerOne.setI(0);
    pidControllerOne.setD(0);

    climberMotorTwo = new CANSparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
    climberMotorTwo.setInverted(false);
    pidControllerTwo = climberMotorOne.getPIDController();
    pidControllerTwo.setP(1);
    pidControllerTwo.setI(0);
    pidControllerTwo.setD(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setPosition(double n) {
    pidControllerOne.setReference(n, ControlType.kPosition);
    pidControllerTwo.setReference(n, ControlType.kPosition);
  }

  public void climb() {
    setPosition(5); // TODO: determine setpoint for arm all the way up.
  }

  public void release() {
    setPosition(10); // TODO: determine setpoint for arm wound up
  }

  public void stop() {
    climberMotorOne.stopMotor();
    climberMotorTwo.stopMotor();
  }

}
