// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class intake extends SubsystemBase {
public TalonFX intake = new TalonFX(IntakeConstants.intakeID);

  public intake() {
    brakeMode();
  }

  public void move(double speed){
  intake.set(speed);
  }

  public Command intakeAuto(double speed){
    return run(
      () -> 
      intake.set(speed)
    );
  }

  
public void brakeMode(){
  intake.setNeutralMode(NeutralModeValue.Coast);
}
  @Override
  public void periodic() {

  }

}
