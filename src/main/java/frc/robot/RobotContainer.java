// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.util.joystick.JoystickMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.arm.ArmPivot;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.arm.ArmSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem;
  public final ArmSubsystem armSubsystem;
  public final Joystick driverJoystick;

  public PathPlannerPath selectedTrajectory;

  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
    this.armSubsystem = new ArmSubsystem();
    this.driverJoystick = new Joystick(0);

    configureAuto();
    configureBindings();
  }

  private void configureAuto() {
    this.selectedTrajectory = PathPlannerPath.fromPathFile("test");
  }

  private void configureBindings() {
    this.swerveSubsystem
        .setDefaultCommand(new JoystickDrive(swerveSubsystem, () -> driverJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS),
            () -> driverJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS),
            () -> driverJoystick.getRawAxis(JoystickMap.RIGHT_X_AXIS)));
    new JoystickButton(driverJoystick, JoystickMap.BUTTON_Y).onTrue(new ArmPivot(armSubsystem, -27.095));
    new JoystickButton(driverJoystick, JoystickMap.BUTTON_A).onTrue(new ArmPivot(armSubsystem, -15));

  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test");

    AutoBuilder.configureHolonomic(swerveSubsystem::getCurrentPose, swerveSubsystem::resetPose,
        swerveSubsystem::getSpeeds, swerveSubsystem::setStates,
        new HolonomicPathFollowerConfig(new PIDConstants(0), new PIDConstants(0),
            Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond, Constants.DriveConstants.kBaseRadius,
            new ReplanningConfig()),
        swerveSubsystem);

    return AutoBuilder.followPathWithEvents(path);
    // return Commands.print("Nothing here");
  }
}
