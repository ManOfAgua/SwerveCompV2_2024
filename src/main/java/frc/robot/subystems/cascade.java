// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CascadeConstants;

public class cascade extends SubsystemBase {
TalonFX cascade = new TalonFX(CascadeConstants.cascadeID);
  public cascade() {
    brakeMode();
  }

  public void move(double speed){
  cascade.set(speed);
  }
  
public void brakeMode(){
  cascade.setNeutralMode(NeutralModeValue.Coast);
}
  @Override
  public void periodic() {

  }

}
