// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public IntakeIOInputs inputs;
  public IntakeIO io;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeIO io) {
    this.inputs = new IntakeIOInputs();
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

}
