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
            //region A button`
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    robot.intake();
                }
            } else { // Reset the 'A' button press flag
                a_pressed = false;
            }

            //region B button`
            if (gamepad1.b) {
                if (!b_pressed) {
                    b_pressed = true;
                    robot.outtake();
                }
            } else { // Reset the 'X' button press flag
                b_pressed = false;
            }

            //region x button`
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
                }
            } else {
                rb_pressed = false;
            }

            //region arm
            double currentAngle = robot.angleSensor.getVoltage();
            double handPosition = Range.scale(currentAngle,2.8,0.5, 0, 1);
            robot.handServo.setPosition(handPosition);
            if (gamepad1.left_trigger > 0 && currentAngle < 2.8d) { //raise arm
                robot.armMotor.setPower(gamepad1.left_trigger);
                robot.stopRoller();
            } else if (gamepad1.right_trigger > 0 && currentAngle > 0.7d) { //lower arm
                robot.armMotor.setPower(-gamepad1.right_trigger);
            } else {
                robot.armMotor.setPower(0);
            }
            //endregion

            //region drivetrain control
            double drive = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x * 0.6d;
            double side = gamepad1.right_stick_x;

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
            telemetry.addData("Arm", "%4.2f", currentAngle);
            telemetry.update();
        }
    }
}
