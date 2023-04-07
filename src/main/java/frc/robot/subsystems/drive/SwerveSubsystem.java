// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.modules.WPI_SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  /** Creates a new DrivetrainSubsystem. */
  public SwerveSubsystem() {
    this.frontRightModule = new WPI_SwerveModule(0, 0, 0, false, false, 0, null, getName());
    this.frontLeftModule = new WPI_SwerveModule(0, 0, 0, false, false, 0, null, getName());
    this.backRightModule = new WPI_SwerveModule(0, 0, 0, false, false, 0, null, getName());
    this.backLeftModule = new WPI_SwerveModule(0, 0, 0, false, false, 0, null, getName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
