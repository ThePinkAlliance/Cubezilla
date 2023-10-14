// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.util.joystick.JoystickMap;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem;
  public final Joystick driverJoystick;

  public PathPlannerPath selectedTrajectory;

  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
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
  }

  public Command getAutonomousCommand() {
    // return new FollowPathWithEvents(
    // new FollowPathHolonomic(
    // selectedTrajectory,
    // this::getPose, // Robot pose supplier
    // this::resetPose, // Method to reset odometry (will be called if your auto has
    // a starting pose)
    // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // this::driveRobotRelative, // Method that will drive the robot given ROBOT
    // RELATIVE ChassisSpeeds
    // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
    // likely live in your Constants
    // // class
    // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    // 4.5, // Max module speed, in m/s
    // 0.4, // Drive base radius in meters. Distance from robot center to furthest
    // module.
    // new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    // ),
    // this // Reference to this subsystem to set requirements
    // ),
    // path, // FollowPathWithEvents also requires the path
    // this::getPose // FollowPathWithEvents also requires the robot pose supplier
    // );
    return Commands.print("Nothing here");
  }
}
