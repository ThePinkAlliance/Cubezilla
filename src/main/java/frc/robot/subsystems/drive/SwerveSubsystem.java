// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.drive.modules.WPI_SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  SwerveDriveKinematics kinematics;

  /** Creates a new DrivetrainSubsystem. */
  public SwerveSubsystem() {
    this.frontRightModule = new WPI_SwerveModule(DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveEncoderReversed, DriveConstants.kFrontRightTurningReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.frontLeftModule = new WPI_SwerveModule(DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveEncoderReversed, DriveConstants.kFrontLeftTurningReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.backRightModule = new WPI_SwerveModule(DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveEncoderReversed, DriveConstants.kBackRightTurningReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.backLeftModule = new WPI_SwerveModule(DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveEncoderReversed, DriveConstants.kBackLeftTurningReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, ModuleConstants.kSteerGains, "base");

    this.kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2),
        new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2),
        new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kWheelBase / 2),
        new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kWheelBase / 2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
