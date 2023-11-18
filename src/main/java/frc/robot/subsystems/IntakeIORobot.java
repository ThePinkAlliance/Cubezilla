package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.littletonrobotics.junction.Logger;

public class IntakeIORobot extends IntakeIO {
    TalonFX intakeMotor;

    public IntakeIORobot() {
        this.intakeMotor = new TalonFX(20);
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        IntakeIOInputs.intakeVelocity = intakeMotor.getSelectedSensorVelocity();

        Logger.getInstance().processInputs("Intake", inputs);
    }

    @Override
    public void setVelocity(double velocity) {
        intakeMotor.set(ControlMode.Velocity, velocity);
    }

}
