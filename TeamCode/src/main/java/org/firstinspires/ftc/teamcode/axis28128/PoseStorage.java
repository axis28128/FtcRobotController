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
 * {@link #loadIfFresh()} in init: it returns the saved pose only if it was written
 * recently enough that an autonomous period actually just ran; otherwise it returns
 * null and the teleop falls back to its hardcoded corner pose. Teleop calls
 * {@link #clear()} when it ends so a second back-to-back teleop also gets the fallback.
 */
public final class PoseStorage {
    private static final String FILE_NAME = "autoEndPose.txt";

    /** A saved pose older than this means no auto just ran (covers auto->teleop transition + setup). */
    public static long MAX_POSE_AGE_MS = 3 * 60 * 1000;

    private PoseStorage() {}

    private static File file() {
        // Resolves to /sdcard/FIRST/settings/autoEndPose.txt on the Robot Controller.
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save(Pose pose) {
        try {
            String data = String.format(Locale.US, "%f %f %f %d",
                    pose.getX(), pose.getY(), pose.getHeading(), System.currentTimeMillis());
            ReadWriteFile.writeFile(file(), data);
        } catch (RuntimeException e) {
            // Never let a failed pose write take down the OpMode.
        }
    }

    /** @return the saved pose if one was written within {@link #MAX_POSE_AGE_MS}, else null. */
    public static Pose loadIfFresh() {
        try {
            File f = file();
            if (!f.exists()) return null;
            String[] parts = ReadWriteFile.readFile(f).trim().split("\\s+");
            if (parts.length != 4) return null;
            long savedAt = Long.parseLong(parts[3]);
            long age = System.currentTimeMillis() - savedAt;
            if (age < 0 || age > MAX_POSE_AGE_MS) return null;
            return new Pose(Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2]));
        } catch (RuntimeException e) {
            return null; // corrupt/unreadable file -> behave like no auto ran
        }
    }

    public static void clear() {
        try {
            File f = file();
            if (f.exists()) //noinspection ResultOfMethodCallIgnored
                f.delete();
        } catch (RuntimeException e) {
            // ignore
        }
    }
}
