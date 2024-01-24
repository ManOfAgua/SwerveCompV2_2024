// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.disabled.Disable;
import frc.robot.subystems.arm;
import frc.robot.subystems.intake;
import frc.robot.subystems.shooter;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  // private boolean driveVal = false;


  /* Setting up bindings for necessary control of the swerve drive platform */
 // My joystick
  private final PS5Controller driver = new PS5Controller(0);
  private final PS5Controller operator = new PS5Controller(1);
  
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain


                              /* Driver Buttons */
  private final JoystickButton resetHeadingButton = new JoystickButton(driver, ControllerConstants.b_L1);
  private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton tri = new JoystickButton(driver, ControllerConstants.b_TRI); //Might be Robot Centric??
  private final JoystickButton intakeButton = new JoystickButton(driver, ControllerConstants.b_R1);
  private final JoystickButton shootButton = new JoystickButton(driver, ControllerConstants.b_X);
  private final JoystickButton intakeRevButton = new JoystickButton(driver, ControllerConstants.b_SQR);
  private final JoystickButton robotCentricButton = new JoystickButton(driver, ControllerConstants.b_MIC);
  private final JoystickButton cascadeButton = new JoystickButton(driver, 0);

                              /*Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final JoystickButton sourceButton = new JoystickButton(operator, 0);
  private final JoystickButton ampButton = new JoystickButton(operator, 0);



                              /* Subsystems */
  private final arm armSub = new arm();
  private final shooter shooterSub = new shooter();
  private final intake intakeSub = new intake();


                              /* Commands */
  private final ManualArmCommand armFwdCommand = new ManualArmCommand(0.15, armSub);
  private final ManualArmCommand armBckCommand = new ManualArmCommand(-0.15, armSub);
  private final ShooterCommand shootCommand = new ShooterCommand(1, shooterSub);
  private final IntakeCommand intakeCommand = new IntakeCommand(0.4, intakeSub);
  private final IntakeCommand intakeRevCommand = new IntakeCommand(-0.2, intakeSub);
  private final ShooterCommand shootRevCommand = new ShooterCommand(-0.65, shooterSub);
  private final ArmPIDCommand armSourceCommand = new ArmPIDCommand(1, armSub);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  // private final SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    

    brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
    tri.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    resetHeadingButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    
    /*Buttons */
        shootButton.whileTrue(shootCommand);
        intakeButton.whileTrue(intakeCommand);
        intakeRevButton.whileTrue(intakeRevCommand);
        intakeRevButton.whileTrue(shootRevCommand);
        armFwdButton.whileTrue(armFwdCommand);
        armBckButton.whileTrue(armBckCommand);

  }

  public RobotContainer() {
    configureBindings();


  }

  public Command getAutonomousCommand() {
            SendableChooser<String> val = (SendableChooser)SmartDashboard.getData("Auton Chooser");
        switch (val.getSelected()) {
            // case "Straight":
            //     return new Straight(s_Swerve);
            // case "Cones":
            //     return new Cones(s_Swerve);
            // case "ConesCurve":
            //     return new ConesCurve(s_Swerve);
            // case "ConesCurve2": //ConesCurve that actualy works
            //     return new ConesCurve2(s_Swerve);
        default:
            return null;
        }
  }

      public Command getDisableCommand() {
            return new Disable(drivetrain);
        }
}
