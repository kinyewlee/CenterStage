package org.firstinspires.ftc.team15091;

public class AutonomousOptions {
    public enum ParkLocation {
        LEFT, // left side backdrop
        RIGHT, // right side backdrop
        NONE // do not attempt to park
    }
    public enum PathLocation {
        WALL, // goes through the first truss, near the wall; collides with robots that do not move
        STAGEDOOR, // goes through the stage door in the center
        NONE // do not attempt to score
    }
    public ParkLocation parkLocationLeft;
    public ParkLocation parkLocationCenter;
    public ParkLocation parkLocationRight;
    public PathLocation pathLocationLeft = PathLocation.STAGEDOOR;
    public PathLocation pathLocationCenter = PathLocation.STAGEDOOR;
    public PathLocation pathLocationRight = PathLocation.STAGEDOOR;
    public boolean parkOnly = false;
    public long delayStartMs = 0;
    public double speedMultiplier = 1;
    public void setParkLocation (PixelPosition location, ParkLocation value) {
        switch (location) {
            case Left:
                parkLocationLeft = value;
                return;
            case Middle:
                parkLocationCenter = value;
                return;
            case Right:
                parkLocationRight = value;
                return;
            case All:
                parkLocationLeft = value;
                parkLocationCenter = value;
                parkLocationRight = value;
                break;
        }
    }
    public void setPathLocation (PixelPosition location, PathLocation value) {
        switch (location) {
            case Left:
                pathLocationLeft = value;
                return;
            case Middle:
                pathLocationCenter = value;
                return;
            case Right:
                pathLocationRight = value;
                return;
            case All:
                pathLocationLeft = value;
                pathLocationCenter = value;
                pathLocationRight = value;
                break;
        }
    }
}
