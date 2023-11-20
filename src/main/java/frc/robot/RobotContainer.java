// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.util.joystick.JoystickMap;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.drive.IntakeSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem;
  public final IntakeSubsystem intakeSubsystem;

  public final Joystick driverJoystick;
  public final Joystick towerJoystick;

  public PathPlannerPath selectedTrajectory;

  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem(Constants.DriveConstants.kDriveKinematics);
    this.intakeSubsystem = new IntakeSubsystem();
    this.driverJoystick = new Joystick(0);
    this.towerJoystick = new Joystick(1);

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

    new JoystickButton(towerJoystick, JoystickMap.RIGHT_BUMPER).onTrue(new Intake(intakeSubsystem, -0.3))
        .onFalse(new Intake(intakeSubsystem, 0));
    new JoystickButton(towerJoystick, JoystickMap.LEFT_BUMPER).onTrue(new Intake(intakeSubsystem, 1))
        .onFalse(new Intake(intakeSubsystem, 0));

  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test");

    return AutoBuilder.followPathWithEvents(path);
    // return Commands.print("Nothing here");
  }
}
