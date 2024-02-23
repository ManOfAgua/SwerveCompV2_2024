// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
public TalonFX intake = new TalonFX(IntakeConstants.intakeID);

  public Intake() {
    brakeMode();
    currentlimit();
  }

  public void move(double speed){
  intake.set(speed);
  }

  public void currentlimit(){
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    intake.getConfigurator().refresh(currentLimits);

    currentLimits.SupplyCurrentLimit = 50;
    currentLimits.SupplyCurrentThreshold = 60;
    currentLimits.SupplyTimeThreshold = 0.1;
    currentLimits.SupplyCurrentLimitEnable = true;
    intake.getConfigurator().apply(currentLimits);
  }

  public Command intakeAuto(double speed){
    return run(
      () -> 
      intake.set(speed)
      )
     .finallyDo(() -> move(0));
  }

  

  
public void brakeMode(){
  intake.setNeutralMode(NeutralModeValue.Coast);
}
  @Override
  public void periodic() {

  }

}
