// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private double MaxSpeed = GeneratedConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 2 * Math.PI;

  /* Joysticks */
  private final PS5Controller driver = new PS5Controller(ControllerConstants.driver);
  private final PS5Controller operator = new PS5Controller(ControllerConstants.operator);

  private final CommandSwerveDrivetrain drivetrain = GeneratedConstants.DriveTrain; // My drivetrain

  /* Driver Buttons */
  private final JoystickButton dr_sqr = new JoystickButton(driver, ControllerConstants.b_SQR);
  private final JoystickButton pointButton = new JoystickButton(driver, ControllerConstants.b_X);
  private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
  private final JoystickButton dr_o = new JoystickButton(driver, ControllerConstants.b_O);
  private final JoystickButton reseedButton = new JoystickButton(driver, ControllerConstants.b_TRI);
  private final JoystickButton dr_L1 = new JoystickButton(driver, ControllerConstants.b_L1);
  private final JoystickButton revintakeButton = new JoystickButton(driver, ControllerConstants.b_R1);
  private final JoystickButton intakeButton = new JoystickButton(driver, ControllerConstants.b_R2);

  private final POVButton dr_0 = new POVButton(driver, 0);
  private final POVButton dr_90 = new POVButton(driver, 90);
  private final POVButton dr_180 = new POVButton(driver, 180);
  private final POVButton dr_270 = new POVButton(driver, 270);

  /* Operator Buttons */
  private final JoystickButton armFwdButton = new JoystickButton(operator, ControllerConstants.b_L2);
  private final JoystickButton armBckButton = new JoystickButton(operator, ControllerConstants.b_R2);
  private final JoystickButton armspeakerCloseButton = new JoystickButton(operator, ControllerConstants.b_TRI);
  private final JoystickButton shootButton = new JoystickButton(operator, ControllerConstants.b_O);
  private final JoystickButton shootSlowButton = new JoystickButton(operator, ControllerConstants.b_SQR);

  private final POVButton armAmpButton = new POVButton(operator, 180);
  private final POVButton photonCommandButton = new POVButton(operator, 90); // TODO: Choose Button

  /* Subsystems */
  private final Arm armSub = new Arm(drivetrain);
  private final Shooter shooterSub = new Shooter();
  private final Intake intakeSub = new Intake();

  SendableChooser<Command> chooser = new SendableChooser<>();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.FieldCentricFacingAngle angleDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.15)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  PhoenixPIDController anglePID = new PhoenixPIDController(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD);

  private void configureBindings() {
    // drivetrain.registerTelemetry(logger::telemeterize);

    angleDrive.HeadingController = anglePID;
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              if (Math.abs(driver.getRightX()) > 0.15) {
                drivetrain.disableHoldHeading();

                return drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate);
              } else {
                drivetrain.enableHoldHeading();

                return angleDrive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withTargetDirection(drivetrain.getHoldHeading());
              }
            }).finallyDo(drivetrain::disableHoldHeading));

    brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));

    pointButton.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    reseedButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /* Driver Commands */
    intakeButton.whileTrue(new IntakeCommand(IntakeConstants.intakeSpd, intakeSub));
    revintakeButton.whileTrue(new IntakeCommand(-IntakeConstants.intakeSpd, intakeSub));
    revintakeButton.whileTrue(new ShooterCommand(-0.2, shooterSub));

    /* Operator Commands */
    shootButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSpd, shooterSub));
    shootSlowButton.whileTrue(new ShooterCommand(ShooterConstants.shooterSlwSpd, shooterSub));

    armFwdButton.whileTrue(new ManualArmCommand(ArmConstants.armSpd, armSub));
    armBckButton.whileTrue(new ManualArmCommand(-ArmConstants.armSpd, armSub));

    armspeakerCloseButton.onTrue(new ArmPIDCommand(20, armSub));

    // photonCommandButton.onTrue(new PhotonCommand(armSub));

    /* Sysid Commands */
    // dr_x.and(dr_0).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    // dr_x.and(dr_180).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    // dr_o.and(dr_0).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    // dr_o.and(dr_180).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    // dr_sqr.and(dr_0).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    // dr_sqr.and(dr_180).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    // dr_tri.and(dr_0).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    // dr_tri.and(dr_180).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("Lower Arm", new ArmPIDCommand(18, armSub));
    NamedCommands.registerCommand("Lower Arm Ground", new ArmPIDCommand(-4, armSub));
    NamedCommands.registerCommand("Raise Arm", new ArmPIDCommand(20, armSub));

    NamedCommands.registerCommand("Intake", intakeSub.intakeAuto(0.5).withTimeout(1));
    NamedCommands.registerCommand("Pickup", intakeSub.intakeAuto(0.2).withTimeout(1.6));
    NamedCommands.registerCommand("PickupSource", intakeSub.intakeAuto(0.4).withTimeout(1));
    NamedCommands.registerCommand("PickupMid", intakeSub.intakeAuto(0.4).withTimeout(4.5));
    NamedCommands.registerCommand("PickupAmp", intakeSub.intakeAuto(0.4).withTimeout(4.5));

    NamedCommands.registerCommand("Intake2", intakeSub.intakeAuto(0.5).withTimeout(1));

    NamedCommands.registerCommand("Shoot", shooterSub.shootAuto(0.7).withTimeout(1.5));
    NamedCommands.registerCommand("Shoot2", shooterSub.shootAuto(0.7).withTimeout(1.5));

    configureBindings();
    // configureDashboard();

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(chooser);
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivetrain::seedFieldRelative));
  }

  // private void configureDashboard() {
  //   /**** Driver tab ****/
  //   var driverTab = Shuffleboard.getTab("Driver");
  //   driverTab.add(new HttpCamera("PhotonCamera", "http://photonvision.local:5800/"))
  //       .withWidget(BuiltInWidgets.kCameraStream)
  //       .withProperties(Map.of("showCrosshair", true, "showControls", false, "rotation", "QUARTER_CCW"))
  //       .withSize(4, 6).withPosition(0, 0);
  // }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}