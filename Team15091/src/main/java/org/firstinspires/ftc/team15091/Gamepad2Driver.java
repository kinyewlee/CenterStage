package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Gamepad")
public class Gamepad2Driver extends GamepadOpMode{
    Gamepad getDriverGamepad() { return gamepad2; }
}
