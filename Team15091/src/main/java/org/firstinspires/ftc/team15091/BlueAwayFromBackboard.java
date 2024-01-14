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
        setupAndWait();
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
            robotDriver.gyroDrive(0.4, 12, 0, 3, null);
            robotDriver.gyroDrive(0.3, 24, 45, 3, null);

            if (should_park) {
                robotDriver.gyroDrive(0.4, -10, 45, 3, null); // move backward
                robotDriver.gyroTurn(0.3, 90, 3);
                robotDriver.gyroSlide(0.3, -37.5, 90, 5, null);
                if (should_score) {
                    armUp.start();
                    robotDriver.gyroDrive(speed_multiplier, 72, 90, 10, null); // move forward
                    robotDriver.gyroSlideAprilTag(0.2, 32, 90, 7, aprilTagDetector, 1);
                    robotDriver.gyroDrive(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    robotDriver.gyroDrive(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                    robotDriver.gyroSlide(0.8, -34, 90, 3, null);
                    robotDriver.gyroDrive(0.7, 22.5, 90, 3, null);
                }
                else {
                    robotDriver.gyroDrive(speed_multiplier, 90, 90, 15, null);
                }
            }
        }
        else if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.4d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.3d, 25d, -45, 3, null);
            robot.togglePixelHolder(true); // release pixel
            if (should_park) {
                sleep(500);
                robotDriver.gyroDrive(0.4, -10, -45, 3, null); // move backward
                robotDriver.gyroTurn(0.3d, 0, 5); // turn left
                //robotDriver.gyroSlide(0.3, 4, 0, 3, null);
                robotDriver.gyroDrive(0.3d, 37.5, 0, 3, null); // drive forward one tile
                robotDriver.gyroTurn(0.3d, 90, 5); // turn left another 90 deg
                if (should_score) {
                    armUp.start();
                    robotDriver.gyroDrive(speed_multiplier, 72, 90, 10, null); // move forward
                    robotDriver.gyroSlideAprilTag(0.2, 26, 90, 7, aprilTagDetector, 3);
                    robotDriver.gyroDrive(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    robotDriver.gyroDrive(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                    robotDriver.gyroSlide(0.8, -22.5, 90, 3, null);
                    robotDriver.gyroDrive(0.7, 22.5, 90, 3, null);
                }
                else {
                    robotDriver.gyroDrive(speed_multiplier, 90, 90, 15, null);
                }
            }
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.4d, 29.5, 0, 3, null); // move forward 1 1/2 tiles, placing the pixel at the spike mark
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            if (should_park) {
                robotDriver.gyroDrive(0.4d, -6, 0, 3, null); // move back, releasing the pixel
                robotDriver.gyroSlide(0.3d, -20, 0, 3, null); // slide one tile to the right
                robotDriver.gyroDrive(0.4d, 26, 0, 3, null); // drive one tile forward
                robotDriver.gyroTurn(0.3d, 90, 5); // turn left
                if (should_score) {
                    armUp.start();
                    robotDriver.gyroDrive(speed_multiplier, 92, 90, 10, null); // move forward and park
                    robotDriver.gyroSlideAprilTag(0.2, 26, 90, 7, aprilTagDetector, 2);
                    robotDriver.gyroDrive(0.2, 15, 90, 3, null);
                    while (armUp.isAlive()) {
                        idle();
                    }
                    robot.openBowl();
                    sleep(1000);
                    robotDriver.gyroDrive(0.4, -5, 90, 3, null);
                    robot.armServo.setPosition(0.8d);
                    robot.closeBowl();
                    sleep(600);
                    armDown.start();
                    robotDriver.gyroSlide(0.8, -29, 90, 3, null);
                    robotDriver.gyroDrive(0.7, 22.5, 90, 3, null);
                }
                else {
                    robotDriver.gyroDrive(speed_multiplier, 120, 90, 15, null);
                }
            }
        }
    }
}
