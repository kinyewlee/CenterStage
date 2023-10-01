package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Example", preselectTeleOp = "Gamepad")
//@Disabled
public class Example extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        setupAndWait();
        robot.armServo.setPosition(0.4d);
        sleep(500);
        robotDriver.gyroDrive(1d, 50d, 0d, 5d, null);
        //release to bucket+
        robot.roller.setPosition(0);
        sleep(3000);
        robot.roller.setPosition(0.5);
        sleep(500);
    }
}

