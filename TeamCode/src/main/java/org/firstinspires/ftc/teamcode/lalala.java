package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class lalala extends LinearOpMode {
    DcMotor m;
    public void runOpMode() throws InterruptedException {
        m = hardwareMap.get(DcMotor.class, "M5");
        waitForStart();
        while (!isStopRequested())
            m.setPower(gamepad2.right_stick_y);
            }
}
