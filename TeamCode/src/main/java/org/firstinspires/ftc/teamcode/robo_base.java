package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class robo_base extends LinearOpMode {

    //Motores
    DcMotor gm;

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    DcMotor garra;
    CRServo inTAKE;

    Servo pulse;

    DcMotor arm;
    DcMotor extensor;

    int maxPosition = -1000;
    int stateROTOR = 1;

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
        inTAKE = hardwareMap.get(CRServo.class, "S1");
        pulse = hardwareMap.get(Servo.class, "S2");
        arm = hardwareMap.get(DcMotor.class, "M5");
        extensor = hardwareMap.get(DcMotor.class,"M6");

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
            if (gamepad2.x) {
                gm.setPower(-spdGarra);
            } else if (gamepad2.b) {
                gm.setPower(spdGarra);

            }
        } else {
            gm.setPower(0);
        }

    }

    public void garraComandos(){
        // In Take
        if(gamepad2.a){
            stateROTOR+=1;
        }

        if(stateROTOR % 2 == 0){
            inTAKE.setPower(1);
        } else {
            inTAKE.setPower(0);
        }

        // Pulso
        if(gamepad2.dpad_right) {
            pulse.setPosition(90);

        } else if (gamepad2.dpad_left) {
            pulse.setPosition(0);
        }

        // Braço
        if(gamepad2.left_trigger > 0) {
            arm.setPower(gamepad2.left_trigger * -0.4);
        }
        else if(gamepad2.right_trigger > 0) {
            arm.setPower(gamepad2.right_trigger * 0.4);
        }

        // Extensor
        if(extensor.getCurrentPosition() < maxPosition){
            extensor.setPower(0);
        } else {
            extensor.setPower(gamepad2.right_stick_y * 0.6);
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
