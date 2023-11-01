package frc.robot.lib;

public abstract class IO<E extends Object> {
    public E inputs;

    public abstract void updateInputs(E inputs);

    public abstract void stop();
}
