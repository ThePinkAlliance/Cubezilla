package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Dashboard {
    private NetworkTable table;
    private NetworkTableInstance ntInstance;

    public Dashboard(String name) {
        this.ntInstance = NetworkTableInstance.getDefault();
        this.table = ntInstance.getTable(name);

        Shuffleboard.getTab(name).buildInto(table, ntInstance.getTable(name + "_test"));
    }

    public Object getObject(String key) {
        return this.table.getEntry(key).getValue().getValue();
    }

    public void putObject(String key, Object o) {
        this.table.getEntry(key).setValue(o);
    }
}
