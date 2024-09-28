package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class robo_base extends LinearOpMode {

    //Motores
    DcMotor gm;

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    DcMotor garra;

    int maxPosition = -3200;
    int midPosition = -2000;
    int encrementoViper;
    int minPosition = 0;
    int constante= 50;

    double spdGarra = 0.4;
    double horinzontal = 0;
    double vertical = 0;
    double pivot = 0 ;

    @Override
    public void runOpMode() throws InterruptedException {
        RF  = hardwareMap.get(DcMotor.class, "M3");
        RB  = hardwareMap.get(DcMotor.class, "M1");
        LF  = hardwareMap.get(DcMotor.class, "M4");
        LB  = hardwareMap.get(DcMotor.class, "M2");
        gm  = hardwareMap.get(DcMotor.class, "M5");
        garra = hardwareMap.get(DcMotor.class, "M6");
        gm.setPower(0.5);
        sleep(3000);
        gm.setPower(0);
        sleep(500);
        gm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(opModeIsActive()){
            garraComandos();

            viperComandos();

            movimentacao();

            telemetry.addData("Garra:",  gm.getCurrentPosition());
            telemetry.update();

        }

    }

    public void viperComandos() {

        if(gm.getCurrentPosition() > maxPosition) {
            if (gamepad2.dpad_up) {
                encrementoViper -= constante;
                gm.setTargetPosition(encrementoViper);
                gm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gm.setPower(-spdGarra);
            } else if (gamepad2.dpad_down) {
                encrementoViper += constante;
                gm.setTargetPosition(encrementoViper);
                gm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gm.setPower(-spdGarra);

            }
        } else {
            gm.setTargetPosition(maxPosition);
            gm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gm.setPower(-spdGarra);
        }








    }

    public void garraComandos(){
        if(gamepad2.left_trigger > 0) {
            garra.setPower(gamepad2.left_trigger * -0.4);
        }
        else if(gamepad2.right_trigger > 0) {
            garra.setPower(gamepad2.right_trigger * 0.4);
        }






    }

    public void movimentacao(){
        horinzontal = -gamepad1.left_stick_y;
        vertical = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        RF.setPower(-pivot + (-vertical + horinzontal));
        RB.setPower(pivot + (-vertical - horinzontal));
        LF.setPower(-pivot + (-vertical - horinzontal));
        LB.setPower(pivot + (-vertical + horinzontal));




    }
}
