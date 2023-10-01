package org.firstinspires.ftc.team15091;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    protected PixelPosition pixelPos = PixelPosition.Left;
    protected VisionPortal visionPortal;
    protected AprilTagDetector aprilTagDetector;
    protected RBProcessor rbProcessor;
    protected long delay_start = 0;
    public boolean should_park = true;
    public boolean should_score = true;
    public double speed_multiplier = 1;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);
        aprilTagDetector = new AprilTagDetector();
        rbProcessor = new RBProcessor();
        aprilTagDetector.init();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagDetector.aprilTag)
                .addProcessor(rbProcessor)
                .build();

        telemetry.addData("Heading", "%.4f", () -> robot.getHeading());
        if (Math.abs(robot.getHeading()) > 20) {
            robot.beep(1);
        }
        telemetry.addLine("Pixel | ")
                .addData("pos", "%s", () -> pixelPos.toString());
        telemetry.addLine("Sensor | ")
                .addData("limit", () -> String.format("%s", robot.limitSwitch.getState()))
                .addData("distance", "%.1f cm", () -> robot.frontSensor.getDistance(DistanceUnit.CM));
        telemetry.addLine("Arm | ")
                .addData("pos", "%s", () -> robot.armPosition);
        telemetry.addLine("Autonomous | ")
                .addData("will go to park", String.format("%b", should_park));

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    delay_start += 100;
                    telemetry.addLine("Variables | ")
                            .addData("Delay ms:", String.format("%d", delay_start));
                }
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    if (delay_start > 0) {
                        delay_start -= 100;
                        telemetry.addLine("Variables | ")
                                .addData("Delay ms:", String.format("%d", delay_start));
                    }
                }
            }

            else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    speed_multiplier = Math.max(0, speed_multiplier - 0.1);
                    telemetry.addLine("Variables | ")
                            .addData("Forward speed:", String.format("%.2f", speed_multiplier));
                }
            }

            else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    speed_multiplier = Math.min(1, speed_multiplier + 0.1);
                    telemetry.addLine("Variables | ")
                            .addData("Forward speed:", String.format("%.2f", speed_multiplier));
                }
            }

            else {
                dpad_pressed = false;
            }

            if (gamepad1.x) {
                if (!x_pressed) {
                    x_pressed = true;
                    should_park = !should_park;
                    telemetry.addLine("Autonomous | ")
                            .addData("will go to park", String.format("%b", should_park));
                    telemetry.update();
                }
            } else {
                x_pressed = false;
            }

            if (gamepad1.y) {
                if (!y_pressed) {
                    y_pressed = true;
                    should_score = !should_score;
                    telemetry.addLine("Autonomous | ")
                            .addData("will go to score", String.format("%b", should_score));
                    telemetry.update();
                }
            } else {
                y_pressed = false;
            }

            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    rbProcessor.add_sat_check = !rbProcessor.add_sat_check;
                    telemetry.addLine("Autonomous | ")
                            .addData("sat check", String.format("%b", rbProcessor.add_sat_check));
                    telemetry.update();
                }
            } else {
                a_pressed = false;
            }

            pixelPos = rbProcessor.position;
            telemetry.update();
            idle();
        }

        if (delay_start > 0d) {
            sleep(delay_start);
        }
    }
}
