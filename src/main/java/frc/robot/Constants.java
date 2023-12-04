// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.Gains;

/** Add your docs here. */
public class Constants {
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.09);
    public static final double kDriveMotorGearRatio = 1 / 5.14;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
        * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.47;
    public static final Gains kSteerGains = new Gains(.45, 0.02, 0);
  }

  public static final class OIConstants {
    public static final double kJoystickDeadband = 0.05;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23.75);

    public static final double kBaseRadius = Math.sqrt((kTrackWidth * kTrackWidth) + (kWheelBase * kWheelBase));

    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    /**
     * HolonomicPathFollowerConfig gives pathplanner information about the
     * drivetrain and
     * specifies PID controllers in x & y axies.
     */
    public static HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(0),
        new PIDConstants(0),
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond, DriveConstants.kBaseRadius,
        new ReplanningConfig());

    public static final int kFrontLeftDriveMotorPort = 15;
    public static final int kBackLeftDriveMotorPort = 17;
    public static final int kFrontRightDriveMotorPort = 11;
    public static final int kBackRightDriveMotorPort = 13;

    public static final int kFrontLeftTurningMotorPort = 16;
    public static final int kBackLeftTurningMotorPort = 18;
    public static final int kFrontRightTurningMotorPort = 12;
    public static final int kBackRightTurningMotorPort = 14;

    public static final int armIntakeMotorPort = 20;
    public static final int LeftArmRotateMotor = 21;
    public static final int RightArmRotateMotor = 22;

    public static final boolean kFrontLeftTurningReversed = false;
    public static final boolean kBackLeftTurningReversed = false;
    public static final boolean kFrontRightTurningReversed = false;
    public static final boolean kBackRightTurningReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    /**
     * These values where determined by lining up all the wheels and recording the
     * outputed positions.
     */
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.98 + 3.14;// -0.09;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -2.26 - 3.14;// 0.94;// 4.05 - 0.20;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.71 + 3.14;// 0.71;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.79 - 3.14;// 5.27;

    // This is the max speed without load.
    public static final double kPhysicalMaxSpeedMetersPerSecond = 2.98704;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; // 0.96
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        / 2.8;
    public static double kTeleDriveSpeedReduction = 1;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kTeleDriveMaxSpeedMetersPerSecond * 0.8;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.5;
  }
}
