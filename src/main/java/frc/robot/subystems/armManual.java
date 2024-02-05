// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class armManual extends SubsystemBase {
public TalonFX leftArm = new TalonFX(ArmConstants.leftarmID);
public TalonFX rightArm = new TalonFX(ArmConstants.rightarmID);

Follower follower = new Follower(ArmConstants.leftarmID, true);

  public armManual() {
    brakeMode();
    rightArm.setControl(follower);
  }

  public void move(double speed){
  leftArm.set(speed);
  }
  
public void brakeMode(){
  leftArm.setNeutralMode(NeutralModeValue.Brake);
  rightArm.setNeutralMode(NeutralModeValue.Brake);
}
  @Override
  public void periodic() {

  }

}
