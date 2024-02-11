// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subystems.arm;
import frc.robot.subystems.intake;
import frc.robot.subystems.shooter;

public class RobotContainer {
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 2 * Math.PI; // 1 rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */

  

                              /* Joysticks */
 private final PS5Controller driver = new PS5Controller(ControllerConstants.driver);
  private final PS5Controller operator = new PS5Controller(ControllerConstants.operator);
  
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain



                              /* Driver Buttons */
  private final JoystickButton dr_sqr = new JoystickButton(driver, ControllerConstants.b_SQR);
  private final JoystickButton dr_x = new JoystickButton(driver, ControllerConstants.b_X);
  private final JoystickButton dr_o = new JoystickButton(driver, ControllerConstants.b_O);
  private final JoystickButton dr_tri = new JoystickButton(driver, ControllerConstants.b_TRI);
  private final JoystickButton dr_L1 = new JoystickButton(driver, ControllerConstants.b_L1);
  private final JoystickButton dr_R1 = new JoystickButton(driver, ControllerConstants.b_R1);
  private final JoystickButton dr_L2 = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton dr_R2 = new JoystickButton(driver, ControllerConstants.b_R2);

  private final POVButton dr_0 = new POVButton(driver, 0);
  private final POVButton dr_180 = new POVButton(driver, 180);


                              /*Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final POVButton armAmpButton = new POVButton(operator, 180);
  // private final POVButton armSourceButton = new POVButton(operator, 0);
  private final JoystickButton armspeakerFarButton = new JoystickButton(operator, ControllerConstants.b_TRI);
  private final JoystickButton armspeakerCloseButton = new JoystickButton(operator, ControllerConstants.b_X);
  private final POVButton photonCommandButton = new POVButton(operator, 90); //TODO: Choose Button

  private final JoystickButton shootButton = new JoystickButton(operator, ControllerConstants.b_O);
  private final JoystickButton shootSlowButton = new JoystickButton(operator, ControllerConstants.b_SQR);


                              /* Subsystems */
  private final arm armSub = new arm();
  private final shooter shooterSub = new shooter();
  private final intake intakeSub = new intake();

  SendableChooser<Command> chooser = new SendableChooser<>();

 
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
    // dr_L2.whileTrue(drivetrain.applyRequest(() -> brake));

  // dr_x.whileTrue(drivetrain
  //       .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    dr_o.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

                                        /*Driver Commands*/
        dr_R2.whileTrue(new IntakeCommand(IntakeConstants.intakeSpd, intakeSub));
        dr_R1.whileTrue(new IntakeCommand(-IntakeConstants.intakeSpd, intakeSub));
        dr_R1.whileTrue(new ShooterCommand(-0.2, shooterSub));

                                        /*Sysid Commands*/

        // dr_x.and(dr_0).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
        // dr_x.and(dr_180).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));
    
        // dr_o.and(dr_0).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
        // dr_o.and(dr_180).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));
    
        // dr_sqr.and(dr_0).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
        // dr_sqr.and(dr_180).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));
    
        // dr_tri.and(dr_0).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
        // dr_tri.and(dr_180).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));






                                        /*Operator Commands*/
        shootButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSpd, shooterSub));
        shootSlowButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSlwSpd, shooterSub));

        armFwdButton.whileTrue(new ManualArmCommand(ArmConstants.armSpd, armSub));
        armBckButton.whileTrue(new ManualArmCommand(-ArmConstants.armSpd, armSub));

        // armSourceButton.onTrue(new ArmPIDCommand(81.5, armSub));
        // armspeakerFarButton.onTrue(new ArmPIDCommand(10, armSub)); //negative is forward
        armspeakerCloseButton.onTrue(new ArmPIDCommand(26, armSub));
        armAmpButton.onTrue(new ArmPIDCommand(105, armSub));

        // photonCommandButton.onTrue(new PhotonCommand(armSub));



  }

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", shooterSub.shootAuto(0.5));
    NamedCommands.registerCommand("Raise Arm", new ArmPIDCommand(10, armSub));
    NamedCommands.registerCommand("Intake", intakeSub.intakeAuto(0.5).withTimeout(5));
    configureBindings();

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(chooser);
    drivetrain.zeroGyro();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivetrain::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}