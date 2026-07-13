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

    // Human-readable outcome of the last save()/loadAndClear() call, for telemetry —
    // read/write failures here were previously swallowed silently, which made a bad
    // handoff indistinguishable from "no autonomous ran". Surface it instead.
    public static volatile String lastStatus = "(none yet)";

    private PoseStorage() {}

    private static File file() {
        // Resolves to /sdcard/FIRST/settings/autoEndPose.txt on the Robot Controller.
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save(Pose pose) {
        try {
            String data = String.format(Locale.US, "%f %f %f",
                    pose.getX(), pose.getY(), pose.getHeading());
            File f = file();
            ReadWriteFile.writeFile(f, data);
            // writeFile() swallows IOExceptions internally, so confirm the write by
            // reading back rather than trusting a lack of thrown exception.
            String readBack = ReadWriteFile.readFile(f);
            lastStatus = data.equals(readBack)
                    ? ("saved OK: " + data)
                    : ("WRITE FAILED (readback mismatch), wanted: " + data + " got: " + readBack);
        } catch (RuntimeException e) {
            lastStatus = "WRITE FAILED (" + e + ")";
        }
    }
    // claude magic

    /** @return the saved pose if the file exists and parses, else null. Deletes the file either way. */
    public static Pose loadAndClear() {
        File f = file();
        try {
            if (!f.exists()) {
                lastStatus = "no file found at " + f.getAbsolutePath();
                return null;
            }
            String raw = ReadWriteFile.readFile(f);
            String[] parts = raw.trim().split("\\s+");
            if (parts.length != 3) {
                lastStatus = "file corrupt, ignored (raw: \"" + raw + "\")";
                return null;
            }
            Pose pose = new Pose(Double.parseDouble(parts[0]),
                    Double.parseDouble(parts[1]),
                    Double.parseDouble(parts[2]));
            lastStatus = "loaded OK: " + raw.trim();
            return pose;
        } catch (RuntimeException e) {
            lastStatus = "READ FAILED (" + e + ")";
            return null; // corrupt/unreadable file -> behave like no auto ran
        } finally {
            //noinspection ResultOfMethodCallIgnored
            f.delete();
        }
    }
}
