// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
public class arm extends SubsystemBase {      

  TalonFX leftArm = new TalonFX(ArmConstants.armID);
    //  PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");


  public arm() {
    brakeMode();
  }

   public double armTickToDegrees() {
    double motorRotations = leftArm.getPosition().getValueAsDouble() / (ArmConstants.kCountsPerRev * ArmConstants.kArmGearRatio);
    double armTicksPerDegree = motorRotations * ArmConstants.kArmScaleFactor;

  return -armTicksPerDegree;
   }

  public void move(double speed){
    leftArm.set(speed);

  }

  private void resetEncoders(){
    leftArm.setPosition(0);
  }
  
  public void brakeMode(){
        leftArm.setNeutralMode(NeutralModeValue.Brake);
  }

    //  public double calculateAngle(){
        
    //     final double camera_Height = Units.inchesToMeters(2);
    //     final double target_Height = Units.inchesToMeters(55.125);
    //     final double camera_Pitch = Units.degreesToRadians(0);
    //     double noTarget_Angle = 0;
    //     double horzoffSet = Units.inchesToMeters(5);
    //     double vertoffSet = Units.inchesToMeters(24.875);
        
    //     var result = photonCamera.getLatestResult();
    //     boolean hasTargets = result.hasTargets();
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     int targetID = target.getFiducialId();

    //     if(hasTargets && (targetID == 7 || targetID == 4)){
    //         System.out.println("\n\nTarget Success\n\n");
    //         double range = PhotonUtils.calculateDistanceToTargetMeters(
    //         camera_Height, 
    //         target_Height, 
    //         camera_Pitch, 
    //         Units.degreesToRadians(result.getBestTarget().getPitch()));
    //         double Radangle = Math.atan((target_Height+vertoffSet)/(range-horzoffSet));
    //         double revolutions = Units.radiansToDegrees(Radangle) * armTickToDegrees();
    //         return revolutions;
    //     }

    //     else{
    //     System.out.println("Target not found");
    //         return noTarget_Angle;
    //     }
    // }

  @Override
  public void periodic() {


 
    //Test 1: -121.3916015625
    //Test 2: -122.13525390625
    //Test 3: -122.5244140625
    //Test 4: -119.46533203125
    //Test 5: -122.77490234375
    // Avg: -121.65830078125
                    
          //////////////////////////// PHOTON         /////////////////////

    // SmartDashboard.putNumber("ID Detected", photonCamera.getLatestResult().getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Yaw April Tag", photonCamera.getLatestResult().getBestTarget().getYaw());
    // SmartDashboard.putNumber("Calculating Camera Angle", calculateAngle());



    // This method will be called once per scheduler run
  }
}