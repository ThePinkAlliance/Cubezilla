package frc.robot.subsystems;

import com.ThePinkAlliance.core.simulation.ctre.CtrePhysicsSim;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

public class IntakeIORobot extends IntakeIO {
    TalonFX intakeMotor;
    TalonFXSimCollection simCollection;

    public IntakeIORobot() {
        this.intakeMotor = new TalonFX(20);
    }

    @Override
    public void stop() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocity = intakeMotor.getSelectedSensorVelocity();

        simCollection.setBusVoltage(RobotController.getBatteryVoltage());

        Logger.getInstance().processInputs("Intake", inputs);
    }

    @Override
    public void setVelocity(double velocity) {
        intakeMotor.set(ControlMode.Velocity, velocity);
    }

}
