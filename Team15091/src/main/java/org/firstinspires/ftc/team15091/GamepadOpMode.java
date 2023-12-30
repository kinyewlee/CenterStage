package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Gamepad (1 Driver)")
public class GamepadOpMode extends OpModeBase {
    Gamepad getDriverGamepad() { return gamepad1; }

    @Override
    public void runOpMode() throws InterruptedException {
        int armLimit = 2400;

        boolean dpadLeftRightPressed = false;
        double rollerVelocity = 0.0; // Initial velocity
        boolean isRollerRunning = false; // Track whether the roller is running
        boolean isWinchRunning = false; // Track whether the winch is running
        int currentWinchPosition = 0;
        boolean loweringInProgress = false; // Initialize a flag to track lowering state
        boolean raisingInProgress = false;
        long liftStartTime = System.currentTimeMillis();
        int droneSequence = 0;
        double driveScale = 0.6;
        double divisor = 2;

        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.liftMotor.getCurrentPosition();
            boolean limitSwitch = robot.limitSwitch.getState();

            if (gamepad1.left_trigger > 0d) { // raise lift
                if (!limitSwitch) { // touched
                    robot.setArmPosition(0.7d);
                } else {
                    rollerVelocity = 0;
                    isRollerRunning = false;
                }
                robot.liftMotor.setTargetPosition(armLimit);
                robot.setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(gamepad1.left_trigger);
                loweringInProgress = false; // Reset the lowering state
                raisingInProgress = false;
            } else if (gamepad1.right_trigger > 0d &&  // lower lift
                    limitSwitch) { // Check if the limit switch is not pressed
                double powerScale = currentArmPosition > 800 ? 1d : 0.3d;
                robot.setLiftMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-gamepad1.right_trigger * powerScale);
                loweringInProgress = false; // Reset the lowering state
                raisingInProgress = false;
            } else {
                if (!loweringInProgress && !raisingInProgress) { // stop lift
                    robot.liftMotor.setPower(0d);
                } else if (loweringInProgress) {
                    double elapsedTime = (System.currentTimeMillis() - liftStartTime) / 1000.0; // Convert to seconds
                    robot.setLiftMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    robot.liftMotor.setPower(elapsedTime > 1d ? -0.8d : -0.1d);
                }

                if (!limitSwitch && !raisingInProgress) {
                    if (loweringInProgress) {
                        loweringInProgress = false;
                        robot.setArmPosition(1d);
                    }
                    robot.setLiftMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            if (gamepad1.x) {
                if (!x_pressed) {
                    x_pressed = true;
                    robot.closeBowl();
                    robot.setArmPosition(0.7d);
                    loweringInProgress = !loweringInProgress;
                    raisingInProgress = false;
                    liftStartTime = System.currentTimeMillis();
                }
            } else { // Reset the 'X' button press flag
                x_pressed = false;
            }
            //endregion

            //region a button
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    if (limitSwitch) {
                        robot.toggleArm();
                    }
                }
            } else {
                a_pressed = false;
            }
            //endregion

            //region b button
            if (gamepad1.b) {
                if (!b_pressed) {
                    b_pressed = true;
                    if (limitSwitch && robot.armPosition == 0) {
                        robot.toggleBowl();
                    }
                }
            } else {
                b_pressed = false;
            }
            //endregion

            //region y button
            if (gamepad1.y) {
                if (!y_pressed) {

                    y_pressed = true;
                    robot.liftMotor.setTargetPosition((int)Math.floor(armLimit / divisor));
                    robot.setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.setArmPosition(0.7d);
                    // robot.liftMotor.setPower(1);
                    loweringInProgress = false;
                    raisingInProgress = true;
                    if (divisor - 0.1 >= 1) {
                        divisor -= 0.1;
                    }
            } else {
                    y_pressed = false;
                    if (!raisingInProgress && !loweringInProgress) { // stop lift
                        robot.liftMotor.setPower(0d);
                    } else if (raisingInProgress) {
                        //robot.liftMotor.setTargetPosition(armLimit / 2);
                        //robot.setLiftMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        robot.liftMotor.setPower(0.8);
                    }
                }      //robot.setLiftMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
            //endregion

            //region roller
            if (gamepad1.left_bumper) {
                if (!lb_pressed) {
                    if (isRollerRunning) {
                        // If the roller is running, stop it
                        rollerVelocity = 0.0;
                        isRollerRunning = false;
                    } else if (!limitSwitch) {
                        // If the roller is stopped, start it with positive velocity
                        rollerVelocity = 1600.0;

                        isRollerRunning = true;
                        robot.setArmPosition(0.9);
                    }
                    lb_pressed = true;
                }
            } else {
                lb_pressed = false;
            }

            if (gamepad1.right_bumper) {
                if (!rb_pressed) {
                    if (isRollerRunning) {
                        // If the roller is running, stop it
                        rollerVelocity = 0.0;
                        isRollerRunning = false;
                    } else {
                        // If the roller is stopped, start it with negative velocity
                        rollerVelocity = -1000.0;
                        isRollerRunning = true;
                    }
                    rb_pressed = true;
                }
            } else {
                rb_pressed = false;
            }

            robot.rollerMotor.setVelocity(rollerVelocity);
            //endregion

            //region winch
            if (gamepad1.dpad_up || (gamepad1.dpad_down && !robot.winchSwitch.isPressed())) {
                if (!dpad_pressed) {
                    if (!isWinchRunning) {
                        if (gamepad1.dpad_up) {
                            robot.winchMotor.setTargetPosition(13400); // extend
                            robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.winchMotor.setPower(1d);
                        } else if (gamepad1.dpad_down && !robot.winchSwitch.isPressed()) {
                            if (robot.winchMotor.getCurrentPosition() > 5000) {
                                robot.winchMotor.setTargetPosition(5000); // hang
                                robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                robot.winchMotor.setPower(1d);
                            } else {
                                robot.winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                robot.winchMotor.setPower(-1d);
                            }
                        }
                        isWinchRunning = true;
                    } else {
                        currentWinchPosition = robot.winchMotor.getCurrentPosition();
                        isWinchRunning = false;
                    }
                    dpad_pressed = true;
                }
            } else {
                if (robot.winchSwitch.isPressed()) {
                    if (isWinchRunning) {
                        robot.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        currentWinchPosition = 0;
                        isWinchRunning = false;
                    }
                } else if (!isWinchRunning) {
                    robot.winchMotor.setTargetPosition(currentWinchPosition);
                    robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.winchMotor.setPower(1d);
                }

                dpad_pressed = false;
            }
            //endregion

            //region drone
            if (gamepad1.dpad_left) {
                if (!dpadLeftRightPressed) {
                    dpadLeftRightPressed = true;
                    robot.railServo.setPosition(0.4);
                    robot.droneServo.setPosition(0);
                    droneSequence = 0;
                }
            } else if (gamepad1.dpad_right) {
                if (!dpadLeftRightPressed) {
                    dpadLeftRightPressed = true;
                    switch (droneSequence) {
                        case 0:
                            robot.railServo.setPosition(0.7);
                            break;
                        case 3:
                            robot.beep();
                            break;
                        case 4:
                            robot.droneServo.setPosition(1);
                            break;
                        case 5:

                            break;
                    }
                    droneSequence++;
                }
            } else {
                dpadLeftRightPressed = false;
            }
            //endregion

            //region drivetrain control
            double leftStickY = -getDriverGamepad().left_stick_y;  // Invert if necessary
            double rightStickY = -getDriverGamepad().right_stick_y;  // Invert if necessary
            double drive;

            if (leftStickY >= 0 && rightStickY >= 0) {
                // Both inputs are positive, so choose the maximum positive value.
                drive = Math.max(leftStickY, rightStickY);
            } else if (leftStickY <= 0 && rightStickY <= 0) {
                // Both inputs are negative, so choose the minimum negative value.
                drive = Math.min(leftStickY, rightStickY);
            } else {
                // The inputs have different signs, so set drive to the sum of both.
                drive = leftStickY + rightStickY;
            }

            if (Math.abs(drive) > 0.5) {
                // Adjust driveScale or perform other actions as needed
                if (driveScale < 1.0) {
                    driveScale += 0.01; // Adjust the increment value as needed
                }
            } else {
                // Reset driveScale or perform other actions as needed when |drive| is not greater than 0.5
                driveScale = 0.6;
            }
            // Apply the scaling factor (0.6 in this case):
            drive *= driveScale;

            double turn = getDriverGamepad().left_stick_x * 0.6d;
            double side = getDriverGamepad().right_stick_x * 0.8d;

            double leftFrontPower = drive + turn + side;
            double leftBackPower = drive + turn - side;
            double rightFrontPower = drive - turn - side;
            double rightBackPower = drive - turn + side;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            robot.setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            //endregion

            // region guide button
            if (gamepad1.guide) {
                if (!guide_pressed) {
                    guide_pressed = true;

                    telemetry.clear();
                    telemetry.addLine("motor | ")
                            .addData("lf", String.format("%.1f", leftFrontPower))
                            .addData("lr", String.format("%.1f", leftBackPower))
                            .addData("rf", String.format("%.1f", rightFrontPower))
                            .addData("rr", String.format("%.1f", rightBackPower));
                    telemetry.addLine("control | ")
                            .addData("drive", String.format("%.1f", drive))
                            .addData("turn", String.format("%.1f", turn))
                            .addData("side", String.format("%.1f", side));
                    telemetry.addLine("servo | ")
                            .addData("bowl", String.format("%.1f", robot.bowlPosition))
                            .addData("arm", String.format("%.1f", robot.armPosition));
                    telemetry.addData("roller", () -> String.format("%.1f", robot.rollerMotor.getVelocity()));
                    telemetry.addLine("arm | ")
                            .addData("pos", () -> String.format("%d", robot.liftMotor.getCurrentPosition()))
                            .addData("amp", () -> String.format("%.1f", robot.liftMotor.getCurrent(CurrentUnit.AMPS)));
                    telemetry.addLine("winch | ")
                            .addData("pos", () -> String.format("%d", robot.winchMotor.getCurrentPosition()))
                            .addData("amp", () -> String.format("%.1f", robot.winchMotor.getCurrent(CurrentUnit.AMPS)));
                    telemetry.addLine("sensor | ")
                            .addData("limit", () -> String.format("%s", robot.limitSwitch.getState()))
                            .addData("winch", () -> String.format("%s", robot.winchSwitch.isPressed()))
                            .addData("front", () -> String.format("%.1f cm", robot.frontSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();
                }
            } else {
                guide_pressed = false;
            }
            // endregion
        }
    }
}
