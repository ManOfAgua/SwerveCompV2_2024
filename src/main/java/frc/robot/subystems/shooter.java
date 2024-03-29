// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooter extends SubsystemBase {
TalonFX shooterTop = new TalonFX(ShooterConstants.shooterTopID);
TalonFX shooterBtm = new TalonFX(ShooterConstants.shooterBtmID);

Follower follower = new Follower(ShooterConstants.shooterTopID, false);

  public shooter() {
    brakeMode();
    shooterTop.setInverted(true);
    shooterBtm.setControl(follower);
  }

  public void move(double speed){
    shooterTop.set(speed);
  }

  public Command shootAuto(double speed){
    return run(
      () -> 
      shooterTop.set(speed)
    );
  }

  public void brakeMode(){
    shooterTop.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
