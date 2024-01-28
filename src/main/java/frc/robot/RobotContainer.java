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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.CascadeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subystems.arm;
import frc.robot.subystems.cascade;
import frc.robot.subystems.intake;
import frc.robot.subystems.shooter;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  // private boolean driveVal = false;  


  /* Setting up bindings for necessary control of the swerve drive platform */

                              /* Joysticks */

 private final PS5Controller driver = new PS5Controller(0);
  private final PS5Controller operator = new PS5Controller(1);
  
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain



                              /* Driver Buttons */
  private final JoystickButton resetHeadingButton = new JoystickButton(driver, ControllerConstants.b_O);
  private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton robotCentricButton = new JoystickButton(driver, ControllerConstants.b_X); //Might be Robot Centric??
  private final JoystickButton intakeButton = new JoystickButton(driver, ControllerConstants.b_R1);
  private final JoystickButton cascadeUpButton = new JoystickButton(driver, ControllerConstants.b_PIC);
  private final JoystickButton cascadeDwnButton = new JoystickButton(driver, ControllerConstants.b_MEN);
  private final JoystickButton armPIDButton = new JoystickButton(driver, ControllerConstants.b_SQR);


                              /*Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final JoystickButton shootButton = new JoystickButton(operator, ControllerConstants.b_X);
  private final JoystickButton shootSlowButton = new JoystickButton(operator, ControllerConstants.b_SQR);


//   private final JoystickButton sourceButton = new JoystickButton(operator, 0);
//   private final JoystickButton ampButton = new JoystickButton(operator, 0);



                              /* Subsystems */
  private final arm armSub = new arm();
  private final shooter shooterSub = new shooter();
  private final intake intakeSub = new intake();
  private final cascade cascadeSub = new cascade();


                              /* Commands */
  private final ManualArmCommand armFwdCommand = new ManualArmCommand(0.3, armSub);
  private final ManualArmCommand armBckCommand = new ManualArmCommand(-0.3, armSub);
  private final ShooterCommand shootCommand = new ShooterCommand(1, shooterSub); //may change speed
  private final IntakeCommand intakeCommand = new IntakeCommand(0.4, intakeSub);
  private final CascadeCommand cascadeUpCommand = new CascadeCommand(0.70, cascadeSub);
  private final CascadeCommand cascadeDwnCommand = new CascadeCommand(-0.70, cascadeSub);
  private final ShooterCommand shootSlowCommand = new ShooterCommand(0.10, shooterSub);

  private final ArmPIDCommand armSourceCommand = new ArmPIDCommand(53.508, armSub);

  SendableChooser<Command> chooser = new SendableChooser<>();

  private Command runAuto = drivetrain.getAutoPath("Auto1");


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    

    brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
    robotCentricButton.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    resetHeadingButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // drivetrain.registerTelemetry(logger::telemeterize);
                                        /*Driver Buttons*/
        



                                        /*Operator Buttons*/
        shootButton.whileTrue(shootCommand);
        intakeButton.whileTrue(intakeCommand);
        armFwdButton.whileTrue(armFwdCommand);
        armBckButton.whileTrue(armBckCommand);
        cascadeUpButton.whileTrue(cascadeUpCommand);
        cascadeDwnButton.whileTrue(cascadeDwnCommand);
        shootSlowButton.whileTrue(shootSlowCommand);
        armPIDButton.onTrue(armSourceCommand);

  }

  public RobotContainer() {
    configureBindings();
    chooser.addOption("Auto 1", runAuto);
    SmartDashboard.putData(chooser);
    
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivetrain::zeroGyro));
      RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivetrain::zeroGyro));


  }


  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
