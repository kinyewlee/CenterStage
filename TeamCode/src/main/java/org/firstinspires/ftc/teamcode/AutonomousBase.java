package org.firstinspires.ftc.teamcode;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);

        telemetry.addData("Heading", "%.4f", robot::getHeading);
        if (Math.abs(robot.getHeading()) > 20) {
            robot.beep(1);
        }

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.update();
            idle();
        }
    }
}
