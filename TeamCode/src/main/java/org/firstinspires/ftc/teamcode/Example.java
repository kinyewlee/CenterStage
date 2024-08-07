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
        robot.armServo.setPosition(1.0);
        sleep(2000);
        robot.intake_motor.setPower(0.6);
        sleep(500);
        robotDriver.gyroDrive(1.0d, 2d, 0d, 5d, null);
        robotDriver.gyroDrive(1.0d, -3d, 0d, 5d, null);
        robotDriver.gyroDrive(1.0d,5d, 0d, 5d, null);
        robotDriver.gyroDrive(1.0d,-4d, 0d, 5d, null);
        sleep(500);
        robot.armServo.setPosition(0.7);
        robot.armServo.setPosition(1.0);
        sleep(500);
        robot.intake_motor.setPower(0);
    }
}

