package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, liftMotor;
    public Servo leftServo, rightServo, rollerServo, handServo, outtakeServo;
    public DigitalChannel limitSwitch;
    private List<DcMotorEx> motors;
    private Context _appContext;
    private int[] beepSoundID = new int[3];
    volatile boolean soundPlaying = false;
    // create a sound parameter that holds the desired player parameters.
    SoundPlayer.PlaySoundParams soundParams = new SoundPlayer.PlaySoundParams(false);

    IMU imu;
    private static final double MAX_VELOCITY = 2800d;
    private static final double COUNTS_PER_MOTOR_REV = 529.2d;    // eg: HD Hex Motor 20:1 560, core hex 288, 40:1 1120
    private static final double DRIVE_GEAR_REDUCTION = 1d;     // This is < 1.0 if geared UP, eg. 26d/10d
    private static final double WHEEL_DIAMETER_INCHES = 2.953d;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359d);
    double slidePosition, handPosition, rollerPosition, outtakePosition;
    private RunMode liftMode;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftFront.setDirection(Direction.REVERSE);
        leftRear.setDirection(Direction.FORWARD);
        rightFront.setDirection(Direction.FORWARD);
        rightRear.setDirection(Direction.FORWARD);

        leftServo = hardwareMap.servo.get("servo_left");
        rightServo = hardwareMap.servo.get("servo_right");
        setSlidePosition(1d, 0d);

        rollerServo = hardwareMap.servo.get("servo_roller");
        setRollerPosition(0.5d);
        handServo = hardwareMap.servo.get("servo_hand");
        handPosition = handServo.getPosition();

        outtakeServo = hardwareMap.servo.get("servo_outtake");
        setOuttake(0d);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit_sensor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setDirection(Direction.FORWARD);
        liftMotor.setCurrentAlert(1d, CurrentUnit.AMPS);
        liftMotor.setPositionPIDFCoefficients(6d);
        liftMotor.setTargetPositionTolerance(2);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setLiftMode(RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        _appContext = hardwareMap.appContext;
        beepSoundID[0] = hardwareMap.appContext.getResources().getIdentifier("beep", "raw", hardwareMap.appContext.getPackageName());
        beepSoundID[1] = hardwareMap.appContext.getResources().getIdentifier("ss_laser", "raw", hardwareMap.appContext.getPackageName());
        beepSoundID[2] = hardwareMap.appContext.getResources().getIdentifier("ss_bb8_up", "raw", hardwareMap.appContext.getPackageName());

        setDriveZeroPowerBehavior(ZeroPowerBehavior.FLOAT);

        beep();
    }

    public final void beep() {
        beep(0);
    }

    final void beep(int beepType) {
        if (!soundPlaying) {
            soundPlaying = true;
            SoundPlayer.getInstance().startPlaying(
                    _appContext,
                    beepSoundID[beepType],
                    soundParams,
                    null,
                    () -> soundPlaying = false);
        }
    }

    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    void resetDrive() {
        setDriveMode(RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(RunMode.RUN_USING_ENCODER);
        setDriveZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    void setDriveZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setDrivePower(double pLeftFront, double pLeftRear, double pRightFront, double pRightRear) {
        leftFront.setPower(pLeftFront);
        leftRear.setPower(pLeftRear);
        rightFront.setPower(pRightFront);
        rightRear.setPower(pRightRear);
    }

    void setDriveVelocity(double pLeftFront, double pLeftRear, double pRightFront, double pRightRear) {
        double vLeftFront = pLeftFront * MAX_VELOCITY;
        double vLeftRear = pLeftRear * MAX_VELOCITY;
        double vRightFront = pRightFront * MAX_VELOCITY;
        double vRightRear = pRightRear * MAX_VELOCITY;
        leftFront.setVelocity(vLeftFront);
        leftRear.setVelocity(vLeftRear);
        rightFront.setVelocity(vRightFront);
        rightRear.setVelocity(vRightRear);
    }

    void setDriveMode(RunMode driveMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(driveMode);
        }
    }

    void setDriveTarget(double distance, boolean moveSideway) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        int dirFL = moveSideway ? -1 : 1;
        int dirFR = 1;
        int dirRL = 1;
        int dirRR = moveSideway ? -1 : 1;

        int leftFrontTarget = leftFront.getCurrentPosition() + moveCounts * dirFL;
        int rightFrontTarget = rightFront.getCurrentPosition() + moveCounts * dirFR;
        int leftRearTarget = leftRear.getCurrentPosition() + moveCounts * dirRL;
        int rightRearTarget = rightRear.getCurrentPosition() + moveCounts * dirRR;

        // Set Target and Turn On RUN_TO_POSITION
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftRear.setTargetPosition(leftRearTarget);
        rightRear.setTargetPosition(rightRearTarget);
    }

    boolean isDriveBusy() {
        return leftFront.isBusy() && leftRear.isBusy() &&
                rightFront.isBusy() && rightRear.isBusy();
    }

    public void setSlidePosition(double newLeftPosition, double newRightPosition) {
        if (slidePosition != newLeftPosition) {
            slidePosition = newLeftPosition;
            leftServo.setPosition(newLeftPosition);
            rightServo.setPosition(newRightPosition);
        }
    }

    public void setRollerPosition(double newRollerPosition) {
        if (rollerPosition != newRollerPosition) {
            rollerPosition = newRollerPosition;
            rollerServo.setPosition(newRollerPosition);
        }
    }

    public void toggleSlide() {
        if (slidePosition == 1d) {
            setSlidePosition(0d, 1d);
        } else {
            setSlidePosition(1d, 0d);
            handServo.setPosition(1d);
        }
        setRollerPosition(0.5d);
    }

    public void intake() {
        if (rollerPosition != 0.5d) {
            setRollerPosition(0.5);
            if (slidePosition == 0d) {
                handServo.setPosition(0.5d);
            }
        } else if (slidePosition == 1d) { // Transfer
            setRollerPosition(0d);
        } else if (slidePosition == 0d) { // Intake
            handServo.setPosition(0d);
            setRollerPosition(1d);
        }
    }

    public void setOuttake(double newOuttakePosition) {
        if (outtakePosition != newOuttakePosition) {
            outtakePosition = newOuttakePosition;
            outtakeServo.setPosition(outtakePosition);
        }
    }
    public void toggleOuttake() {
        if (outtakePosition != 0d) {
            setOuttake(0d);
        } else {
            setOuttake(1d);
        }
    }

    public void setLiftMode(DcMotor.RunMode newMode) {
        if (liftMode != newMode) {
            liftMotor.setMode(newMode);
            liftMode = newMode;
        }
    }
}