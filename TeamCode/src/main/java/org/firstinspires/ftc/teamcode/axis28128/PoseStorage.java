package org.firstinspires.ftc.teamcode.axis28128;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * Hands the robot's pose from an autonomous OpMode to the teleop OpMode through a file
 * on the Robot Controller, so the handoff survives the app restarting between periods.
 *
 * Autonomous calls {@link #save} (periodically and on stop). Teleop calls
 * {@link #loadAndClear()} in init: if the file exists, that means an autonomous period
 * just ran, so it reads the pose and deletes the file. If the file doesn't exist (e.g.
 * a second teleop run back-to-back with no autonomous in between), it returns null and
 * the teleop falls back to its hardcoded corner pose. Deleting the file on read means a
 * second teleop right after never sees a stale pose from the earlier run.
 */
public final class PoseStorage {
    private static final String FILE_NAME = "autoEndPose.txt";

    private PoseStorage() {}

    private static File file() {
        // Resolves to /sdcard/FIRST/settings/autoEndPose.txt on the Robot Controller.
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save(Pose pose) {
        try {
            String data = String.format(Locale.US, "%f %f %f",
                    pose.getX(), pose.getY(), pose.getHeading());
            ReadWriteFile.writeFile(file(), data);
        } catch (RuntimeException e) {
            // Never let a failed pose write take down the OpMode.
        }
    }

    /** @return the saved pose if the file exists, else null. Deletes the file either way. */
    public static Pose loadAndClear() {
        File f = file();
        try {
            if (!f.exists()) return null;
            String[] parts = ReadWriteFile.readFile(f).trim().split("\\s+");
            if (parts.length != 3) return null;
            return new Pose(Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2]));
        } catch (RuntimeException e) {
            return null; // corrupt/unreadable file -> behave like no auto ran
        } finally {
            //noinspection ResultOfMethodCallIgnored
            f.delete();
        }
    }
}
