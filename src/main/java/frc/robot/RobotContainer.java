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
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 2.5 * Math.PI; // 1 rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */

  

                              /* Joysticks */
 private final PS5Controller driver = new PS5Controller(ControllerConstants.driver);
  private final PS5Controller operator = new PS5Controller(ControllerConstants.operator);
  
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain



                              /* Driver Buttons */
  private final JoystickButton resetHeadingButton = new JoystickButton(driver, ControllerConstants.b_O);
  private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton robotCentricButton = new JoystickButton(driver, ControllerConstants.b_X);//Might be Robot Centric??

  private final JoystickButton  intakeButton = new JoystickButton(driver, ControllerConstants.b_R2);
  private final JoystickButton revintakeButton = new JoystickButton(driver, ControllerConstants.b_R1);

  private final JoystickButton sysidQuadFwd = new JoystickButton(driver, ControllerConstants.b_SQR);
    private final JoystickButton sysidQuadRev = new JoystickButton(driver, ControllerConstants.b_TRI);



                              /*Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final POVButton armAmpButton = new POVButton(operator, 180);
  private final POVButton armSourceButton = new POVButton(operator, 0);
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
    brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));

    robotCentricButton.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    resetHeadingButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

                                        /*Driver Commands*/
        intakeButton.whileTrue(new IntakeCommand(IntakeConstants.intakeSpd, intakeSub));
        revintakeButton.whileTrue(new IntakeCommand(-IntakeConstants.intakeSpd, intakeSub));
        revintakeButton.whileTrue(new ShooterCommand(-0.2, shooterSub));

        // sysidQuadFwd.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)); //sqr
        // sysidQuadRev.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); //tri
        // intakeButton.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)); //R2
        // revintakeButton.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)); //R1






                                        /*Operator Commands*/
        shootButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSpd, shooterSub));
        shootSlowButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSlwSpd, shooterSub));

        armFwdButton.whileTrue(new ManualArmCommand(ArmConstants.armSpd, armSub));
        armBckButton.whileTrue(new ManualArmCommand(-ArmConstants.armSpd, armSub));

        armSourceButton.onTrue(new ArmPIDCommand(81.5, armSub));
        armspeakerFarButton.onTrue(new ArmPIDCommand(10, armSub)); //negative is forward
        armspeakerCloseButton.onTrue(new ArmPIDCommand(-10, armSub));
        armAmpButton.onTrue(new ArmPIDCommand(112.3, armSub));

        // photonCommandButton.onTrue(new PhotonCommand(armSub));



  }

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", shooterSub.shootAuto(0.5));
    NamedCommands.registerCommand("Raise Arm", new ArmPIDCommand(10, armSub));
    NamedCommands.registerCommand("Intake", intakeSub.intakeAuto(0.5));
    NamedCommands.registerCommand("IntakeStop", intakeSub.intakeAuto(0));
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