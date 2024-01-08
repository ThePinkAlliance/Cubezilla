// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class SpeedTest extends Command {
  private SwerveSubsystem swerveSubsystem;

  private boolean starting = true;
  private boolean isFinished = false;
  private double lastSpeed = 0;
  private double startingTime;

  /** Creates a new SpeedTest. */
  public SpeedTest(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startingTime = Timer.getFPGATimestamp();

    swerveSubsystem.resetPose(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double nextSpeed = 0;
    double distance = Units.metersToInches(swerveSubsystem.getCurrentPose().getX());

    if (starting) {
      nextSpeed = Math.max(lastSpeed + 0.95, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

      // Max 173in
      if (distance > 153) {
        starting = false;
      }
    } else {
      nextSpeed = Math.min(lastSpeed - 0.95, 0);

      if (nextSpeed <= 0) {
        isFinished = true;
      }
    }

    SmartDashboard.putNumber("Test Distance", distance);
    SmartDashboard.putNumber("Test Speed", nextSpeed);

    swerveSubsystem
        .setStates(new ChassisSpeeds(nextSpeed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setStates(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
