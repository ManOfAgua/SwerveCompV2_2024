// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public double getSensorPositionRaw(){
    return cascade.getPosition().getValueAsDouble();
 }

  public double cascadeTickToDegrees() {
   return 2;
 //   double motorRotations = rightArm.getSelectedSensorPosition() / (2048 * 200);
 //   double armTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor; // 359.489141
 // return armTicksPerDegree;
  }
public void brakeMode(){
  cascade.setNeutralMode(NeutralModeValue.Coast);
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Position Raw Cascade", getSensorPositionRaw());

  }

}
