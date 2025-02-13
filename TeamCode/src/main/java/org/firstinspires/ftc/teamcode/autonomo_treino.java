package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class autonomo_treino extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor M1 = hardwareMap.get(DcMotor.class, "M1");
        DcMotor M2 = hardwareMap.get(DcMotor.class,"M2");
        waitForStart();
        M1.setPower(0.5);
        M2.setPower(-0.5);
        sleep(1000);
        M1.setPower(0);
        M2.setPower(0);
    }
}
