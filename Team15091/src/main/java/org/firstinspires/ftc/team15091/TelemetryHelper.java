package org.firstinspires.ftc.team15091;

public class TelemetryHelper {
    public static String parkLocationAsString (AutonomousOptions.ParkLocation a) {
        switch (a) {
            case LEFT:
                return "left";
            case RIGHT:
                return "right";
            case NONE:
                return "none";
            default:
                return "unknown";
        }
    }

    public static String pathLocationAsString (AutonomousOptions.PathLocation a) {
        switch (a) {
            case WALL:
                return "wall";
            case STAGEDOOR:
                return "stage door";
            case NONE:
                return "none";
            default:
                return "unknown";
        }
    }
}
