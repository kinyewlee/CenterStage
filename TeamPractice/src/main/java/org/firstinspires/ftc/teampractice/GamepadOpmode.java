package org.firstinspires.ftc.teampractice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad")
public class GamepadOpmode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        int highPolePos = 800;
        int grabSequence = 0;
        int autoArm = 0; // Initialize a flag to track auto state

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
            int currentArmPosition = robot.armMotor.getCurrentPosition();
            boolean limitSwitch = robot.limitSwitch.getState();

            if (gamepad1.left_bumper) { // raise lift
                robot.armMotor.setTargetPosition(highPolePos);
                robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(0.3d);
                autoArm = 0; // Reset the lowering state
            } else if (gamepad1.right_bumper && // lower lift
                    robot.limitSwitch.getState() == true) { // when limit sensor not pressed
                double powerScale = currentArmPosition > 400 ? 1d : 0.4d;
                robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-0.3d * powerScale);
                autoArm = 0; // Reset the lowering state
            } else {
                switch (autoArm) {
                    case 0: // stop lift
                        robot.armMotor.setPower(0d);
                        break;
                    case 1: // lower arm
                    case 3: // reset arm
                        robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        robot.armMotor.setPower(-0.3d);
                        break;
                    case 2: // raise arm
                        robot.armMotor.setTargetPosition(highPolePos);
                        robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(0.3d);
                        break;
                }

                if (!limitSwitch) { // limit touched
                    if (autoArm != 2) {
                        robot.setArmMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        if (autoArm == 3) {
                            autoArm = 2;
                        } else {
                            autoArm = 0;
                        }
                    }
                }
            }
            //endregion

            //region grabber servo control
            if (gamepad1.a) {
                if (!a_pressed) {
                    a_pressed = true;
                    switch (grabSequence) {
                        case 0:
                            robot.beep(2);
                            autoArm = 1;
                            grabSequence = 1;
                            break;
                        case 1:
                            robot.beep(3);
                            robot.setGrabber(0.2d);
                            grabSequence = 2;
                            break;
                        case 2:
                            robot.beep(4);
                            autoArm = 2;
                            grabSequence = 3;
                            break;
                        case 3:
                            robot.beep(5);
                            robot.setGrabber(0.8d);
                            grabSequence = 0;
                            break;
                    }
                }
            } else {
                a_pressed = false;
            }

            if (gamepad1.x) {
                if (!x_pressed) {
                    grabSequence = 0;
                    autoArm = 3;
                    robot.setGrabber(0.8d);
                    robot.beep();
                    x_pressed = true;
                }
            } else {
                x_pressed = false;
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

            // Apply the scaling factor (0.6 in this case):
            drive *= 0.4d;
            double turn = gamepad1.left_stick_x * 0.5d;
            double side = gamepad1.right_stick_x * 0.6d;

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

            telemetry.update();
        }
    }
}
