// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

  public static final CommandXboxController driveController = new CommandXboxController(Constants.DrivetrainConstants.driveControllerPort);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    /* FieldOrientedAngularVelocity will set the robot's angular velocity based on the right stick. */
    /* Is field oriented unless right bumper is pressed. */
    Command driveAnglularVelocity = swerveSubsystem.driveCommandAngularVelocity(
        () -> MathUtil.applyDeadband(-driveController.getLeftY(), DrivetrainConstants.stickDeadband),
        () -> MathUtil.applyDeadband(-driveController.getLeftX(), DrivetrainConstants.stickDeadband),
        () -> MathUtil.applyDeadband(-driveController.getRightX(), DrivetrainConstants.stickDeadband),
        () -> driveController.rightBumper().getAsBoolean());

    /* FieldOrientedDirectAngle will set the robot's target angle based on the right stick. */
    /* Is field oriented. */
    Command driveDirectAngle = swerveSubsystem.driveCommandDirectAngle(
      () -> MathUtil.applyDeadband(driveController.getLeftY(), DrivetrainConstants.stickDeadband),
      () -> MathUtil.applyDeadband(driveController.getLeftX(), DrivetrainConstants.stickDeadband),
      () -> driveController.getRightX(),
      () -> driveController.getRightY());

    swerveSubsystem.setDefaultCommand(
      Constants.DrivetrainConstants.useAngularVelocityDriveMode ? 
      driveAnglularVelocity : 
      driveDirectAngle
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("PathPlanner Auto", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    driveController.a().onTrue(Commands.runOnce(swerveSubsystem::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveSubsystem.setMotorBrake(brake);
  }
}
