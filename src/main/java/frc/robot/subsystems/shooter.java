// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
TalonFX shooterTop = new TalonFX(ShooterConstants.shooterTopID);
TalonFX shooterBtm = new TalonFX(ShooterConstants.shooterBtmID);

Follower follower = new Follower(ShooterConstants.shooterTopID, false);
public boolean robotCentric;

  public Shooter() {
    coastMode();
    currentlimit();
    shooterTop.setInverted(true);
    shooterBtm.setControl(follower);
  }

  public void move(double speed){
    shooterTop.set(speed);
  }

  public void currentlimit(){
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    shooterTop.getConfigurator().refresh(currentLimits);
    shooterBtm.getConfigurator().refresh(currentLimits);

    currentLimits.SupplyCurrentLimit = 40;
    currentLimits.SupplyCurrentThreshold = 50;
    currentLimits.SupplyTimeThreshold = 0.1;
    currentLimits.SupplyCurrentLimitEnable = true;
    shooterTop.getConfigurator().apply(currentLimits);
    shooterBtm.getConfigurator().apply(currentLimits);
  }

  public Command shootAuto(double speed){
    return run(
      () -> 
      shooterTop.set(speed)
    )
    .finallyDo(() -> move(0));
  }

  public void coastMode(){
    shooterTop.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
