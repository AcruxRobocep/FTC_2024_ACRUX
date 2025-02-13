package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class movimento_elian extends LinearOpMode {

    DcMotor M1;
    DcMotor M2;
    DcMotor M3;
    DcMotor M4;
    double horinzontal = 0;
    double vertical = 0;
    double pivot =0;
    Servo garra;


    @Override
    public void runOpMode() throws InterruptedException {
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");


        garra = hardwareMap.get(Servo.class, "S1");


        waitForStart();
        while (opModeIsActive()) {
            movimentacao();
            controlegarra();


        }

    }


    public void controlegarra() {

        M3.setPower(gamepad2.right_stick_y / 2);
        M4.setPower(gamepad2.left_stick_y / 2);

        if (gamepad2.dpad_up) {
            garra.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            garra.setPosition(0);

        }
    }


    public void movimentacao() {
        horinzontal = gamepad1.right_stick_y;
        vertical = gamepad1.right_stick_x;

        M1.setPower(vertical - horinzontal);
        M2.setPower(vertical + horinzontal);

    }
}