package frc.robot.subsystems.arm;

import java.security.PublicKey;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax leftSparxMax;
    private CANSparkMax rightSparxMax;
    private ArmInputs inputs;

    private double speedRange = .25;

    public void setDesiredPos(double desiredPos) {
        leftSparxMax.getPIDController().setReference(desiredPos, CANSparkMax.ControlType.kPosition);

        inputs.armSetpoint = desiredPos;
    }

    public ArmSubsystem() {
        this.inputs = new ArmInputs();

        this.leftSparxMax = new CANSparkMax(Constants.DriveConstants.LeftArmRotateMotor, MotorType.kBrushless);
        this.rightSparxMax = new CANSparkMax(Constants.DriveConstants.RightArmRotateMotor, MotorType.kBrushless);

        leftSparxMax.getPIDController().setOutputRange(-speedRange, speedRange);

        leftSparxMax.getEncoder().setPosition(0);
        rightSparxMax.getEncoder().setPosition(0);

        rightSparxMax.follow(leftSparxMax, true);
        leftSparxMax.getPIDController().setP(1);
    }

    public void periodic() {
        double leftArmEncoderValue = leftSparxMax.getEncoder().getPosition();
        double rightArmEncoderValue = rightSparxMax.getEncoder().getPosition();

        SmartDashboard.putNumber("left Arm Neo Encoder Value", (leftArmEncoderValue));
        SmartDashboard.putNumber("Right Arm Neo Encoder Value", (rightArmEncoderValue));

        inputs.armPosition = leftSparxMax.getEncoder().getPosition();

        Logger.getInstance().processInputs("Arm", inputs);
    }
}