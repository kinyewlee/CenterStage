package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.util.ElapsedTime;

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
    protected YellowProcessor yellowProcessor;
    public AutonomousOptions autonomousOptions = new AutonomousOptions();;
    private boolean setParkLocationMode = false;
    private PixelPosition setParkPosition;
    private boolean setPathLocationMode = false;
    private PixelPosition setPathPosition;
    long delayInputTimer = 0;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);
        aprilTagDetector = new AprilTagDetector();
        rbProcessor = new RBProcessor();
        yellowProcessor = new YellowProcessor();
        aprilTagDetector.init();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagDetector.aprilTag)
                .addProcessor(rbProcessor)
                .addProcessor(yellowProcessor)
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
        telemetry.addLine("Options | ")
                .addData("park (left)", () -> TelemetryHelper.parkLocationAsString(autonomousOptions.parkLocationLeft))
                .addData("park (center)", () -> TelemetryHelper.parkLocationAsString(autonomousOptions.parkLocationCenter))
                .addData("park (right)", () -> TelemetryHelper.parkLocationAsString(autonomousOptions.parkLocationRight))
                .addData("path (left)", () -> TelemetryHelper.pathLocationAsString(autonomousOptions.pathLocationLeft))
                .addData("path (center)", () -> TelemetryHelper.pathLocationAsString(autonomousOptions.pathLocationCenter))
                .addData("path (right)", () -> TelemetryHelper.pathLocationAsString(autonomousOptions.pathLocationRight))
                .addData("delay", () -> String.format("%d ms", autonomousOptions.delayStartMs))
                .addData("score yellow pixel", () -> String.format("%b", autonomousOptions.parkOnly));

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    if (setParkLocationMode) {
                        setParkPosition = PixelPosition.Middle;
                    }
                    else if (setPathLocationMode) {
                        setPathPosition = PixelPosition.Middle;
                    }
                }
            }
            else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    if (setParkLocationMode) {
                        setParkPosition = PixelPosition.All;
                    }
                    else if (setPathLocationMode) {
                        setPathPosition = PixelPosition.All;
                    }
                }
            }
            else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    if (setParkLocationMode) {
                        setParkPosition = PixelPosition.Left;
                    }
                    else if (setPathLocationMode) {
                        setPathPosition = PixelPosition.Left;
                    }
                }
            }
            else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (!dpad_pressed) {
                    dpad_pressed = true;
                    if (setParkLocationMode) {
                        setParkPosition = PixelPosition.Right;
                    }
                    else if (setPathLocationMode) {
                        setPathPosition = PixelPosition.Right;
                    }
                }
            }
            else {
                dpad_pressed = false;
            }

            if (gamepad1.x) {
                if (!x_pressed) {
                    x_pressed = true;
                    if (setParkLocationMode && setParkPosition != null) {
                        setParkLocationMode = false;
                        autonomousOptions.setParkLocation(setParkPosition, AutonomousOptions.ParkLocation.LEFT);
                        setParkPosition = null;
                    }
                    else if (setPathLocationMode && setPathPosition != null) {
                        setPathLocationMode = false;
                        autonomousOptions.setPathLocation(setPathPosition, AutonomousOptions.PathLocation.WALL);
                        setPathPosition = null;
                    }
                }
            } else {
                x_pressed = false;
            }

            if (gamepad1.y) {
                if (!y_pressed) {
                    y_pressed = true;
                    if (setPathLocationMode && setPathPosition != null) {
                        setPathLocationMode = false;
                        autonomousOptions.setPathLocation(setPathPosition, AutonomousOptions.PathLocation.STAGEDOOR);
                        setPathPosition = null;
                    }
                }
            } else {
                y_pressed = false;
            }

            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    if (setParkLocationMode && setParkPosition != null) {
                        setParkLocationMode = false;
                        autonomousOptions.setParkLocation(setParkPosition, AutonomousOptions.ParkLocation.NONE);
                        setParkPosition = null;
                    }
                    else if (setPathLocationMode && setPathPosition != null) {
                        setPathLocationMode = false;
                        autonomousOptions.setPathLocation(setPathPosition, AutonomousOptions.PathLocation.NONE);
                        setPathPosition = null;
                    }
                }
            } else {
                a_pressed = false;
            }

            if (gamepad1.b) {
                if (!a_pressed) {
                    b_pressed = true;
                    if (setParkLocationMode && setParkPosition != null) {
                        setParkLocationMode = false;
                        autonomousOptions.setParkLocation(setParkPosition, AutonomousOptions.ParkLocation.RIGHT);
                        setParkPosition = null;
                    }
                }
            } else {
                b_pressed = false;
            }

            if (gamepad1.left_trigger > 0.1) {
                if (System.currentTimeMillis() - delayInputTimer > 10) {
                    delayInputTimer = System.currentTimeMillis();
                    autonomousOptions.delayStartMs -= Math.floor(gamepad1.left_trigger * 10);
                    if (autonomousOptions.delayStartMs < 0) autonomousOptions.delayStartMs = 0;
                }
            }
            if (gamepad1.right_trigger > 0.1) {
                if (System.currentTimeMillis() - delayInputTimer > 10) {
                    delayInputTimer = System.currentTimeMillis();
                    autonomousOptions.delayStartMs += Math.floor(gamepad1.right_trigger * 10);
                }
            }

            if (gamepad1.left_bumper) {
                if (!lb_pressed) {
                    lb_pressed = true;
                    setParkLocationMode = !setParkLocationMode;
                    setParkPosition = null;
                    setPathLocationMode = false;
                }
            } else {
                lb_pressed = false;
            }
            if (gamepad1.right_bumper) {
                if (!rb_pressed) {
                    rb_pressed = true;
                    setPathLocationMode = !setPathLocationMode;
                    setPathPosition = null;
                    setParkLocationMode = false;
                }
            } else {
                rb_pressed = false;
            }

            pixelPos = rbProcessor.position;
            telemetry.update();
            if (setParkLocationMode) {
                telemetry.addLine();
                if (setParkPosition == null) {
                    telemetry.addLine("Setting park position for:");
                    telemetry.addLine("D-Pad Left: Left");
                    telemetry.addLine("D-Pad Up: Center");
                    telemetry.addLine("D-Pad Right: Right");
                    telemetry.addLine("D-Pad Down: All");
                    telemetry.addLine("LB: cancel");
                }
                else {
                    telemetry.addLine("Setting park position for " + setParkPosition.toString());
                    telemetry.addLine("X: park left");
                    telemetry.addLine("B: park right");
                    telemetry.addLine("A: do not park");
                    telemetry.addLine("LB: cancel");
                }
            }
            else if (setPathLocationMode) {
                telemetry.addLine();
                if (setParkPosition == null) {
                    telemetry.addLine("Setting path position for:");
                    telemetry.addLine("D-Pad Left: Left");
                    telemetry.addLine("D-Pad Up: Center");
                    telemetry.addLine("D-Pad Right: Right");
                    telemetry.addLine("D-Pad Down: All");
                    telemetry.addLine("RB: cancel");
                }
                else {
                    telemetry.addLine("Setting path position for " + setPathPosition.toString());
                    telemetry.addLine("X: first truss");
                    telemetry.addLine("Y: stage door");
                    telemetry.addLine("A: do not path");
                    telemetry.addLine("RB: cancel");
                }
            }
            else {
                telemetry.addLine("LB: set park position");
                telemetry.addLine("RB: set path position");
                telemetry.addLine("LT: decrease starting delay");
                telemetry.addLine("RT: increase starting delay");
            }
            idle();
        }

        if (autonomousOptions.delayStartMs > 0d) {
            sleep(autonomousOptions.delayStartMs);
        }
    }

    final protected void gyroDriveWithMultiplier(double maxDriveSpeed,
                                                 double distance,
                                                 double heading,
                                                 double timeoutS,
                                                 IObjectDetector<Boolean> objectDetector) {
        robotDriver.gyroDrive(Math.min(1, maxDriveSpeed * autonomousOptions.speedMultiplier), distance, heading, timeoutS, objectDetector);
    }
    final protected void gyroSlideWithMultiplier(double maxSlideSpeed,
                                                 double distance,
                                                 double heading,
                                                 double timeoutS,
                                                 IObjectDetector<Boolean> objectDetector) {
        robotDriver.gyroSlide(Math.min(1, maxSlideSpeed * autonomousOptions.speedMultiplier), distance, heading, timeoutS, objectDetector);
    }
    final protected void gyroTurnWithMultiplier(double maxTurnSpeed, double heading, double timeoutS) {
        robotDriver.gyroTurn(Math.min(1, maxTurnSpeed * autonomousOptions.speedMultiplier), heading, timeoutS);
    }
}
