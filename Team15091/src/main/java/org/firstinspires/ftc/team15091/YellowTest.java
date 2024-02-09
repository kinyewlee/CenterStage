package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Yellow Test", preselectTeleOp = "Gamepad")
public class YellowTest extends AutonomousBase{

    @Override
    public void runOpMode() throws InterruptedException {
        visionPortal.setProcessorEnabled(yellowProcessor, true);
        visionPortal.setProcessorEnabled(rbProcessor, false);
        visionPortal.setProcessorEnabled(aprilTagDetector.aprilTag, false);
        setupAndWait();
        if (!opModeIsActive()) return;
    }
}
