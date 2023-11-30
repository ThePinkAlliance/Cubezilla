package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryLoader {
    private static String extension = ".path";

    public static Trajectory loadFromPath(String path) {
        Trajectory trajectory;

        if (!path.contains(extension) && !path.contains("output/")) {
            path = "output/" + path + extension;
        }

        if (!path.contains(extension)) {
            path = path + extension;
        }

        try {
            Path trajectoryPath = Filesystem
                    .getDeployDirectory()
                    .toPath()
                    .resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            // Sending it to the Driver Station however im not sure if it's a very
            // noticeable ill that check later
            DriverStation.reportError(
                    "Unable to open trajectory: " + path,
                    ex.getStackTrace());
            // by making it fail loudly, we can catch it before a match starts
            throw new Error("Unable to open trajectory: " + path);
        }

        return trajectory;
    }
}
