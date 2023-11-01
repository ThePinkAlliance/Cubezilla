package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class IntakeIOSim extends IntakeIO {
    WPI_TalonFX intakeMotor;

    public IntakeIOSim() {
        this.intakeMotor = new WPI_TalonFX(20);
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        IntakeIOInputs.intakeVelocity = intakeMotor.getSelectedSensorVelocity();
    }

    @Override
    public void setVelocity(double velocity) {
        intakeMotor.set(ControlMode.Velocity, velocity);
    }

}
