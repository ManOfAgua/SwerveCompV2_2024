// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;


import java.nio.file.Path;

import javax.swing.plaf.TreeUI;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
public class arm extends SubsystemBase {      

   TalonFX leftArm = new TalonFX(ArmConstants.leftarmID);
   TalonFX rightArm = new TalonFX(ArmConstants.rightarmID);
   PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");
   Follower follower = new Follower(ArmConstants.leftarmID, true);

  final double camera_Height = Units.inchesToMeters(8);
  final double target_Height = Units.inchesToMeters(56.125); //Distance from floor to the middle of april tag
  final double camera_Pitch = Units.degreesToRadians(50);

  double horzoffSet = Units.inchesToMeters(5);
  double vertoffSet = Units.inchesToMeters(24.3125); //80.4375 height of middle of speaker opening.. 80.4375-56.125=24.3125


  public arm() {
    brakeMode();
    currentlimit();
    rightArm.setControl(follower);

  }

  public void move(double speed){
    leftArm.set(speed);
    }

  public void currentlimit(){
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    leftArm.getConfigurator().refresh(currentLimits);
    rightArm.getConfigurator().refresh(currentLimits);

    currentLimits.SupplyCurrentLimit = 70;
    currentLimits.SupplyCurrentThreshold = 80;
    currentLimits.SupplyTimeThreshold = 0.1;
    currentLimits.SupplyCurrentLimitEnable = true;
    leftArm.getConfigurator().apply(currentLimits);
    rightArm.getConfigurator().apply(currentLimits);
  }

  public double armTickToDegrees() {
    double motorRotations = leftArm.getPosition().getValueAsDouble() / (ArmConstants.kCountsPerRev * ArmConstants.kArmGearRatio);
    double armTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor;
    double offset = 90;
  return -armTicksPerDegree+offset;
   }

  private void resetEncoders(){
    leftArm.setPosition(0);
    rightArm.setPosition(0);
  }
  
  public void brakeMode(){
    leftArm.setNeutralMode(NeutralModeValue.Brake);
    rightArm.setNeutralMode(NeutralModeValue.Brake);
  }

  public double leftmotorTemp(){
   double armTemp = leftArm.getDeviceTemp().getValueAsDouble();
   return armTemp;

  }


  public double calculateAngle(){      
    var result = photonCamera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    if(target == null){
      return armTickToDegrees();
    }

    else{
    int targetID = target.getFiducialId();
        if(targetID==7||targetID==4){
            double range = PhotonUtils.calculateDistanceToTargetMeters(
            camera_Height, 
            target_Height, 
            camera_Pitch, 
            Units.degreesToRadians(target.getPitch())); //gets the pitch of the april tag
            double angle = Math.asin((target_Height)/(range));
            return angle;
        }
        else{
          return armTickToDegrees();
        }
    }
  }


  @Override
  public void periodic() {


    SmartDashboard.putNumber("Arm Angle", armTickToDegrees());
    SmartDashboard.putNumber("Left Arm Temp", leftmotorTemp());

    // SmartDashboard.putNumber("Photon", calculateAngle());
    // SmartDashboard.putNumber("Photon Angle", calculateAngle());

    //Test 1: -121.3916015625
    //Test 2: -122.13525390625
    //Test 3: -122.5244140625
    //Test 4: -119.46533203125
    //Test 5: -122.77490234375
    // Avg: -121.65830078125 * 2
                    
          //////////////////////////// PHOTON         /////////////////////

    // SmartDashboard.putNumber("ID Detected", photonCamera.getLatestResult().getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Yaw April Tag", photonCamera.getLatestResult().getBestTarget().getYaw());
    // SmartDashboard.putNumber("Calculating Camera Angle", calculateAngle());



    // This method will be called once per scheduler run
  }
}