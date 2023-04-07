package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
  public double getDrivePosition();

  public double getSteerPosition();

  public double getDriveVelocity();

  public double getSteerVelocity();

  public void resetEncoders();

  public double getAbsoluteEncoderAngle();

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public double getRawAbsoluteAngularPosition();

  public void stop();

  public void setDesiredState(SwerveModuleState state);
}
