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
        robotDriver.gyroDrive(0.6d, -19d, 0d, 5d, null);
        //release to bucket+
        sleep(1000);
    }
}

