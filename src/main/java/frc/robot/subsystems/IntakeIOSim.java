package frc.robot.subsystems;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOSim extends IntakeIO {
    WPI_TalonFX intakeMotor;

    public IntakeIOSim() {
        this.intakeMotor = new WPI_TalonFX(20);

        CtrePhysicsSim.getInstance().addTalonFX(intakeMotor, 3, 5800);
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocity = intakeMotor.getSelectedSensorVelocity();

        Logger.getInstance().processInputs("Intake", inputs);
    }

    @Override
    public void setVelocity(double velocity) {
        intakeMotor.set(ControlMode.PercentOutput, velocity);
    }

}
