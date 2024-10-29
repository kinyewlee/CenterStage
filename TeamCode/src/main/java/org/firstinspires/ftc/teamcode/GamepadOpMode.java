package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Gamepad")
public class GamepadOpMode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        double driveScale = 0.6;
        int armLimit = 3500;
        boolean loweringInProgress = false; // Initialize a flag to track lowering state
        boolean raisingInProgress = false;
        long liftStartTime = System.currentTimeMillis();

        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean limitSwitch = robot.limitSwitch.getState();

            //region a button
            if (gamepad1.a || gamepad2.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    robot.toggleSlide();
                }
            } else { // Reset the 'X' button press flag
                a_pressed = false;
            }
            //endregion

            //region b button
            if (gamepad1.b || gamepad2.b) {
                if (!b_pressed) {
                    b_pressed = true;
                    robot.intake();
                }
            } else { // Reset the 'X' button press flag
                b_pressed = false;
            }
            //endregion

            //region x button
            if (gamepad1.x || gamepad2.x) {
                if (!x_pressed) {
                    x_pressed = true;
                    if (limitSwitch) {
                        robot.toggleOuttake();
                    }
                }
            } else { // Reset the 'X' button press flag
                x_pressed = false;
            }
            //endregion

            //region arm control
            int currentArmPosition = robot.liftMotor.getCurrentPosition();
            if (gamepad1.left_trigger > 0d) { // raise lift
                if (!limitSwitch) { // touched
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
                    }
                    robot.setLiftMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            //endregion

            //region left bumper
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                if (!lb_pressed) {
                    lb_pressed = true;
                    robot.setLiftMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.liftMotor.setTargetPosition(armLimit);
                    robot.setLiftMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.setRollerPosition(0.5d);
                    loweringInProgress = false;
                    raisingInProgress = !raisingInProgress;
                }
            } else {
                lb_pressed = false;
                if (!raisingInProgress && !loweringInProgress && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) { // stop lift
                    robot.liftMotor.setPower(0d);
                } else if (raisingInProgress) {
                    robot.liftMotor.setPower(0.8);
                }
            }
            //endregion

            //region right bumper
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                if (!rb_pressed) {
                    rb_pressed = true;
                    robot.setOuttake(0d);
                    loweringInProgress = !loweringInProgress;
                    raisingInProgress = false;
                    liftStartTime = System.currentTimeMillis();
                }
            } else {
                rb_pressed = false;
            }
            //endregion

            //region drivetrain control
            double leftStickY = -gamepad1.left_stick_y;  // Invert if necessary
            double rightStickY = -gamepad1.right_stick_y;  // Invert if necessary
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

            double turn = gamepad1.left_stick_x * 0.6d;
            double side = gamepad1.right_stick_x * 0.8d;;

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

            // Show the elapsed game time and wheel power.
            telemetry.clearAll();
            telemetry.addLine("motor | ")
                    .addData("lf", String.format("%.1f", leftFrontPower))
                    .addData("lr", String.format("%.1f", leftBackPower))
                    .addData("rf", String.format("%.1f", rightFrontPower))
                    .addData("rr", String.format("%.1f", rightBackPower));
            telemetry.addLine("sensor | ")
                    .addData("limit", () -> String.format("%s", robot.limitSwitch.getState()));
            telemetry.update();
        }
    }
}