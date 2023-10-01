package org.firstinspires.ftc.team15091;
// turns: left angles are positive, right angles are negative
// slides: left distances are positive, right distances are negative
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "BlueAwayFromBackboard", preselectTeleOp = "Gamepad")
public class BlueAwayFromBackboard extends AutonomousBase{
    @Override
    public void runOpMode() throws InterruptedException {
        autonomousOptions.parkLocationLeft = AutonomousOptions.ParkLocation.RIGHT;
        autonomousOptions.parkLocationCenter = AutonomousOptions.ParkLocation.RIGHT;
        autonomousOptions.parkLocationRight = AutonomousOptions.ParkLocation.RIGHT;
        setupAndWait();
        if (!opModeIsActive()) return;
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 10, false);
        PixelPosition initialPos = rbProcessor.position;
        Thread armUp = new Thread() {
            public void run() {
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException ie) {}
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setTargetPosition(450);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(robot.liftMotor.isBusy()) {
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
                while(robot.liftMotor.isBusy()) {
                    robot.liftMotor.setPower(0.6);
                    idle();
                }
                robot.liftMotor.setPower(0);
            }
        };
        if (initialPos == PixelPosition.Left) {
            gyroDriveWithMultiplier(0.4, 12, 0, 3, null);
            gyroDriveWithMultiplier(0.3, 24, 45, 3, null);

            if (autonomousOptions.pathLocationLeft == AutonomousOptions.PathLocation.STAGEDOOR) {
                gyroDriveWithMultiplier(0.4, -10, 45, 3, null); // move backward
                gyroTurnWithMultiplier(0.3, 90, 3);
                gyroSlideWithMultiplier(0.3, -37.5, 90, 5, null);
                if (!autonomousOptions.parkOnly) {
                    armUp.start();
                    gyroDriveWithMultiplier(1, 72, 90, 10, null); // move forward
                    robotDriver.gyroSlideAprilTag(0.2, 32, 90, 7, aprilTagDetector, 1);
                    gyroDriveWithMultiplier(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    gyroDriveWithMultiplier(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                }
                else {
                    gyroDriveWithMultiplier(1, 90, 90, 15, null);
                }
            }
            if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 22.5, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
            else if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -34, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
        }
        else if (initialPos == PixelPosition.Right) {
            gyroDriveWithMultiplier(0.4d, 12d, 0, 3, null);
            gyroDriveWithMultiplier(0.3d, 25d, -45, 3, null);
            if (autonomousOptions.pathLocationRight == AutonomousOptions.PathLocation.STAGEDOOR) {
                sleep(500);
                gyroDriveWithMultiplier(0.4, -10, -45, 3, null); // move backward
                gyroTurnWithMultiplier(0.3d, 0, 5); // turn left
                //gyroSlideWithMultiplier(0.3, 4, 0, 3, null);
                gyroDriveWithMultiplier(0.3d, 37.5, 0, 3, null); // drive forward one tile
                gyroTurnWithMultiplier(0.3d, 90, 5); // turn left another 90 deg
                if (!autonomousOptions.parkOnly) {
                    armUp.start();
                    gyroDriveWithMultiplier(1, 72, 90, 10, null); // move forward
                    robotDriver.gyroSlideAprilTag(0.2, 26, 90, 7, aprilTagDetector, 3);
                    gyroDriveWithMultiplier(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    gyroDriveWithMultiplier(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                }
                else {
                    gyroDriveWithMultiplier(1, 90, 90, 15, null);
                }
            }
            if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 34, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
            else if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -22.5, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
        }
        else { // pixel in the middle position
            gyroDriveWithMultiplier(0.4d, 29.5, 0, 3, null); // move forward 1 1/2 tiles, placing the pixel at the spike mark
            sleep(500);
            if (autonomousOptions.pathLocationCenter == AutonomousOptions.PathLocation.STAGEDOOR) {
                gyroDriveWithMultiplier(0.4d, -6, 0, 3, null); // move back, releasing the pixel
                gyroSlideWithMultiplier(0.3d, -20, 0, 3, null); // slide one tile to the right
                gyroDriveWithMultiplier(0.4d, 26, 0, 3, null); // drive one tile forward
                gyroTurnWithMultiplier(0.3d, 90, 5); // turn left
                if (!autonomousOptions.parkOnly) {
                    armUp.start();
                    gyroDriveWithMultiplier(1, 92, 90, 10, null); // move forward and park
                    robotDriver.gyroSlideAprilTag(0.2, 26, 90, 7, aprilTagDetector, 2);
                    gyroDriveWithMultiplier(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    gyroDriveWithMultiplier(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                }
                else {
                    gyroDriveWithMultiplier(1, 120, 90, 15, null);
                }
            }
            if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.LEFT) {
                gyroSlideWithMultiplier(0.8, 29, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
            else if (autonomousOptions.parkLocationLeft == AutonomousOptions.ParkLocation.RIGHT) {
                gyroSlideWithMultiplier(0.8, -29, 90, 3, null);
                gyroDriveWithMultiplier(0.7, 22.5, 90, 3, null);
            }
        }
    }
}
