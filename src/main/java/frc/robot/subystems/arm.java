// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
public class arm extends SubsystemBase {      

   TalonFX leftArm = new TalonFX(ArmConstants.leftarmID);
   TalonFX rightArm = new TalonFX(ArmConstants.rightarmID);
   PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");
   Follower follower = new Follower(ArmConstants.leftarmID, true);

  final double camera_Height = Units.inchesToMeters(2);
  final double target_Height = Units.inchesToMeters(56.125); //Distance from floor to the middle of april tag
  final double camera_Pitch = Units.degreesToRadians(0);

  double noTarget_Angle = 45;
  double horzoffSet = Units.inchesToMeters(5);
  double vertoffSet = Units.inchesToMeters(24.3125); //80.4375 height of middle of speaker opening.. 80.4375-56.125=24.3125


  public arm() {
    brakeMode();
    rightArm.setControl(follower);
  }

  public void move(double speed){
    leftArm.set(speed);
    }

  public double armTickToDegrees() {
    double motorRotations = leftArm.getPosition().getValueAsDouble() / (ArmConstants.kCountsPerRev * ArmConstants.kArmGearRatio);
    double armTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor;
    double offset = 90;
  return -armTicksPerDegree;
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


  // public double calculateAngle(){
  //   var result = photonCamera.getLatestResult();
  //   boolean hasTargets = result.hasTargets();
  //   PhotonTrackedTarget target = result.getBestTarget();
  //   int targetID = target.getFiducialId();

  //     if(hasTargets && (targetID == 7 || targetID == 4)){
  //           System.out.println("\n\nTarget Success\n\n");
  //           double range = PhotonUtils.calculateDistanceToTargetMeters(
  //           camera_Height, 
  //           target_Height, 
  //           camera_Pitch, 
  //           Units.degreesToRadians(result.getBestTarget().getPitch())); //gets the pitch of the april tag
  //           double angle = Math.asin((target_Height+vertoffSet)/range);
  //                       System.out.println(range);
  //                       System.out.println(angle);

  //           return angle;
  //       }

  //       else{
  //       System.out.println("Target not found");
  //           return armTickToDegrees();
  //       }
  //   }


  @Override
  public void periodic() {


    SmartDashboard.putNumber("Arm Angle", armTickToDegrees());
    SmartDashboard.putNumber("Left Arm Temp", leftmotorTemp());
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