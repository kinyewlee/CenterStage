package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Gamepad")
public class GamepadOpMode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region a button
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    robot.toggleGrip();
                }
            } else { // Reset the 'A' button press flag
                a_pressed = false;
            }

            //region b button
            if (gamepad1.b) {
                if (!b_pressed) {
                    b_pressed = true;
                    robot.toggleHand();
                }
            } else { // Reset the 'A' button press flag
                b_pressed = false;
            }

            //region x button
            if (gamepad1.x || gamepad2.x) {
                if (!x_pressed) {
                    x_pressed = true;
                }
            } else { // Reset the 'X' button press flag
                x_pressed = false;
            }
            //endregion

            if (gamepad1.y || gamepad2.y) {
                if (!y_pressed) {
                    y_pressed = true;
                }
            } else { // Reset the 'Y' button press flag
                y_pressed = false;
            }

            //region left bumper
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                if (!lb_pressed) {
                    lb_pressed = true;
                }
            } else {
                lb_pressed = false;
            }
            //endregion

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                if (!rb_pressed) {
                    rb_pressed = true;
                    robot.toggleHandMiddle();
                }
            } else {
                rb_pressed = false;
            }

            if (gamepad1.dpad_up) {
                robot.handUp();
            }

            if (gamepad1.dpad_down) {
                robot.handDown();
            }

            //region lift
            boolean limitSwitch = robot.limitSwitch.getState();
            int currentLiftPosition = robot.liftMotor.getCurrentPosition();
            double newHandPosition = Range.scale(Math.abs(currentLiftPosition), 0, 3000, 0d, 1d);
            if (gamepad1.left_trigger > 0d) { // raise lift
                robot.liftMotor.setTargetPosition(-5000);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0d &&  // lower lift
                    limitSwitch) { // Check if the limit switch is not pressed
                double powerScale = currentLiftPosition > -800 ? 0.3d : 1d;
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(gamepad1.right_trigger * powerScale);
            } else {
                robot.liftMotor.setPower(0d);
                if (!limitSwitch) {
                    robot.liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

//            if (gamepad1.right_trigger > 0d && !limitSwitch) {
//                robot.setGripPosition(0.5d);
//            }

            //region drivetrain control
            double drive = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x * 0.7d;
            double side = gamepad1.right_stick_x;

            if (gamepad1.dpad_left) {
                turn = -0.2d;
            }

            if (gamepad1.dpad_right) {
                turn = 0.2d;
            }

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
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Lift", "%d", currentLiftPosition);
            telemetry.update();
        }
    }
}
