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
    public Servo gripServo, handServo;
    public DigitalChannel limitSwitch;
    private List<DcMotorEx> motors;
    private Context _appContext;
    private int[] beepSoundID = new int[3];
    volatile boolean soundPlaying = false;
    // create a sound parameter that holds the desired player parameters.
    SoundPlayer.PlaySoundParams soundParams = new SoundPlayer.PlaySoundParams(false);

    IMU imu;
    private static final double MAX_VELOCITY = 2800d;
    private static final double COUNTS_PER_MOTOR_REV = 1120d;    // eg: HD Hex Motor 20:1 560, core hex 288, 40:1 1120
    private static final double DRIVE_GEAR_REDUCTION = 0.67d;     // This is < 1.0 if geared UP, eg. 26d/10d
    private static final double WHEEL_DIAMETER_INCHES = 2.953d;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359d);
    double gripPosition, handPosition;
    int handMode;

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        leftFront.setDirection(Direction.REVERSE);
        leftRear.setDirection(Direction.REVERSE);
        rightFront.setDirection(Direction.FORWARD);
        rightRear.setDirection(Direction.FORWARD);

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(Direction.FORWARD);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");

        gripServo = hardwareMap.servo.get("grip");
        gripPosition = gripServo.getPosition();

        handServo = hardwareMap.servo.get("hand");
        handPosition = gripServo.getPosition();

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

    public void setGripPosition(double newGripPosition) {
        if (gripPosition != newGripPosition) {
            gripPosition = newGripPosition;
            gripServo.setPosition(gripPosition);
        }
    }

    public void toggleGrip() {
        if (gripPosition == 0.9d) {
            setGripPosition(0.5d);
        } else {
            setGripPosition(0.9d);
        }
    }

    public void setHandPosition(double newHandPosition) {
        if (handPosition != newHandPosition) {
            handPosition = newHandPosition;
            handServo.setPosition(handPosition);
        }
    }

    public void toggleHand() {
        if (handPosition == 0.42d) {
            setHandPosition(0d);
        } else {
            setHandPosition(0.42d);
        }
    }

    public void toggleHandMiddle() {
        if (handPosition == 0.42d) {
            setHandPosition(1d);
        } else {
            setHandPosition(0.42d);
        }
    }

    public void handDown() {
        if (handPosition < 1d) {
            setHandPosition(handPosition + 0.0025d);
        }
    }

    public void handUp() {
        if (handPosition > 0d) {
            setHandPosition(handPosition - 0.0025d);
        }
    }
}


