package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "RedNearBackboard", preselectTeleOp = "Gamepad")
public class RedNearBackboard extends AutonomousBase {
    // arm servo: 0 is out, 0.8 is in
    // bowl servo: 1 is out, 0 is in
    Thread armUp = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(450);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.8);
                idle();
            }
            robot.liftMotor.setPower(0);
            robot.extendArm();
            robot.setArmPosition(0.7);
            robot.setArmPosition(0);
        }
    };

    Thread armDown = new Thread() {
        public void run() {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftMotor.setTargetPosition(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftMotor.isBusy()) {
                robot.liftMotor.setPower(0.6);
                idle();
            }
            robot.liftMotor.setPower(0);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        autonomousOptions.parkLocationLeft = AutonomousOptions.ParkLocation.RIGHT;
        autonomousOptions.parkLocationCenter = AutonomousOptions.ParkLocation.RIGHT;
        autonomousOptions.parkLocationRight = AutonomousOptions.ParkLocation.RIGHT;
        setupAndWait();
        if (!opModeIsActive()) return;
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor) (hardwareMap.get("sensor_front")), 7, false);
        PixelPosition initialPos = rbProcessor.position;
        robot.setArmPosition(0.7);
        if (initialPos == PixelPosition.Right) {
            gyroDriveWithMultiplier(0.4d, 12d, 0, 3, null);
            gyroDriveWithMultiplier(0.3d, 24.5d, -45, 3, null);
            gyroDriveWithMultiplier(0.4, -10, -45, 3, null); // move backward
            armUp.start();
            gyroTurnWithMultiplier(0.3, -90, 3);
            gyroDriveWithMultiplier(0.4, 22.5, -90, 3, null);
            robotDriver.gyroSlideAprilTag(0.2, 3, -90, 7, aprilTagDetector, 6);
            frontDistance.setThreshold(3);
            robotDriver.gyroDriveAprilTag(0.2, 10, -90, 3, aprilTagDetector, 6, frontDistance);
            while (armUp.isAlive()) {
                idle();
            }
            robot.openBowl();
            sleep(1000);
            gyroDriveWithMultiplier(0.3, -5, -90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.closeBowl();
            sleep(600);
            armDown.start();
            if (autonomousOptions.parkLocationRight == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 34, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
            else if (autonomousOptions.parkLocationRight == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -22.5, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
        } else if (initialPos == PixelPosition.Left) {
            gyroDriveWithMultiplier(0.4d, 12d, 0, 3, null);
            gyroDriveWithMultiplier(0.3d, 25d, 45, 3, null);
            gyroDriveWithMultiplier(0.4, -10, 45, 3, null); // move backward
            armUp.start();
            gyroTurnWithMultiplier(0.3, -90, 3);
            gyroDriveWithMultiplier(0.4, 22.5, -90, 3, null);
            robotDriver.gyroSlideAprilTag(0.2, 15, -90, 7, aprilTagDetector, 4);
            frontDistance.setThreshold(3);
            robotDriver.gyroDriveAprilTag(0.2, 10, -90, 3, aprilTagDetector, 4, frontDistance);
            while (armUp.isAlive()) {
                idle();
            }
            robot.openBowl();
            sleep(1000);
            gyroDriveWithMultiplier(0.3, -5, -90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.closeBowl();
            sleep(600);
            armDown.start();
            if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 22.5, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
            else if (autonomousOptions.parkLocationRight == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -34, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
        } else { // pixel in the middle position
            gyroDriveWithMultiplier(0.4, 29.5, 0, 5, null);
            gyroDriveWithMultiplier(0.3, -7, 0, 5, null);
            armUp.start();
            gyroTurnWithMultiplier(0.3, -90, 5);
            gyroDriveWithMultiplier(0.4, 22.5, -90, 5, null);
            robotDriver.gyroSlideAprilTag(0.2, 10, -90, 7, aprilTagDetector, 5);
            frontDistance.setThreshold(3);
            robotDriver.gyroDriveAprilTag(0.2, 6, -90, 3, aprilTagDetector, 5, frontDistance);
            while (armUp.isAlive()) {
                idle();
            }
            robot.openBowl();
            sleep(1000);
            gyroDriveWithMultiplier(0.3, -5, -90, 3, null);
            robot.armServo.setPosition(0.8d);
            robot.closeBowl();
            sleep(600);
            armDown.start();
            if (autonomousOptions.parkLocationCenter == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 29, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
            else if (autonomousOptions.parkLocationCenter == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -29, -90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, -90, 3, null);
            }
        }
    }
}