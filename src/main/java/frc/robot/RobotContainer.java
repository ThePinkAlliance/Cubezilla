// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake;
import frc.robot.commands.JoystickDrive;
import frc.robot.lib.JoystickMap;
import frc.robot.subsystems.drive.IntakeSubsystem;
import frc.robot.subsystems.drive.PinkHolonomicController;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public final Joystick driverJoystick;
  public final Joystick towerJoystick;

  public PathPlannerPath selectedTrajectory;
  public SendableChooser<PathPlannerPath> chooser;

  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
    this.intakeSubsystem = new IntakeSubsystem();
    this.driverJoystick = new Joystick(0);
    this.towerJoystick = new Joystick(1);
    this.chooser = new SendableChooser<>();

    configureAuto();
    configureBindings();
  }

  private void configureAuto() {
    /*
     * Using redundant fromPath methods in the event a path does not exist it will
     * throw an error on startup instead of doing it before a match.
     */
    this.chooser.addOption("Test", PathPlannerPath.fromPathFile("Test"));
    this.chooser.setDefaultOption("Crazy", PathPlannerPath.fromPathFile("crazy"));
    this.chooser.addOption("Straight", PathPlannerPath.fromPathFile("Straight"));

    SmartDashboard.putData(chooser);
  }

  private void configureBindings() {
    this.swerveSubsystem
        .setDefaultCommand(new JoystickDrive(swerveSubsystem, () -> driverJoystick.getRawAxis(JoystickMap.LEFT_X_AXIS),
            () -> driverJoystick.getRawAxis(JoystickMap.LEFT_Y_AXIS),
            () -> driverJoystick.getRawAxis(JoystickMap.RIGHT_X_AXIS)));

    new JoystickButton(driverJoystick, JoystickMap.BUTTON_START)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.resetGyro()));

    new JoystickButton(towerJoystick, JoystickMap.RIGHT_BUMPER).onTrue(new Intake(intakeSubsystem, -0.3))
        .onFalse(new Intake(intakeSubsystem, 0));
    new JoystickButton(towerJoystick, JoystickMap.LEFT_BUMPER).onTrue(new Intake(intakeSubsystem, 1))
        .onFalse(new Intake(intakeSubsystem, 0));
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = chooser.getSelected();

    // Sets the robot starting position the same as starting point on path.
    Pose2d path_pose = path.getStartingDifferentialPose();
    swerveSubsystem.resetPose(new Pose2d(path_pose.getX(), path_pose.getY(), swerveSubsystem.getRotation()));

    Command command = new FollowPathCommand(path, swerveSubsystem::getCurrentPose, swerveSubsystem::getSpeeds,
        swerveSubsystem::setStates,
        new PinkHolonomicController(new PIDConstants(.8, 0.1, .5), new PIDConstants(1.5, 0, 0.0),
            Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond, Constants.DriveConstants.kBaseRadius),
        new ReplanningConfig(), swerveSubsystem);

    // Added events to the path follower
    return new FollowPathWithEvents(command, path, swerveSubsystem::getCurrentPose);
  }
}
