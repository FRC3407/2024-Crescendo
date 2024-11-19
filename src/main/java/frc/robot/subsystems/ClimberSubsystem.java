// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private static enum ClimberState {
    IDLE, UP, CLIMBING
  };

  private CANSparkMax climberMotorOne;
  private CANSparkMax climberMotorTwo;
  private RelativeEncoder encoderOne;
  private RelativeEncoder encoderTwo;
  private ClimberState climberState = ClimberState.IDLE;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotorOne = new CANSparkMax(Constants.ClimberConstants.climberOneCanID, MotorType.kBrushless);
    climberMotorOne.setInverted(true);
    encoderOne = climberMotorOne.getEncoder();

    climberMotorTwo = new CANSparkMax(Constants.ClimberConstants.climberTwoCanID, MotorType.kBrushless);
    climberMotorTwo.setInverted(false);
    encoderTwo = climberMotorTwo.getEncoder();
  }

  @Override
  public void periodic() {
    if (climberState == ClimberState.UP) {
      if (encoderOne.getPosition() < 5) {
        climberMotorOne.set(0.5);
      } else {
        climberMotorOne.stopMotor();
      }
      if (encoderTwo.getPosition() < 5) {
        climberMotorTwo.set(0.5);
      } else {
        climberMotorTwo.stopMotor();
      }
    }
    if (climberState == ClimberState.CLIMBING) {
      //TODO : drive each motor forward until we've climbed.
    }
  }


  public void armsUp() {
    climberState = ClimberState.UP;
  }

  public void climb() {
    climberState = ClimberState.CLIMBING;
  }

  public void stop() {
    climberMotorOne.stopMotor();
    climberMotorTwo.stopMotor();
  }

}
