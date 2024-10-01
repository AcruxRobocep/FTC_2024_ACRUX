package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class robo_base extends LinearOpMode {



    // Movimentação
    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    double horinzontal = 0;
    double vertical = 0;
    double pivot = 0 ;

    // Viper-Slide
    DcMotor viper;
    int maxPositionViper = -3300;

    // Garra In-Take
    DcMotor extensor;
    CRServo inTAKE;
    Servo pulse;
    int maxPosition = -1000;
    int clockROTOR = 0;
    int unclockROTOR = 0;




    double spdGarra = 0.4;


    @Override
    public void runOpMode() throws InterruptedException {
        RF  = hardwareMap.get(DcMotor.class, "M3");
        RB  = hardwareMap.get(DcMotor.class, "M1");
        LF  = hardwareMap.get(DcMotor.class, "M4");
        LB  = hardwareMap.get(DcMotor.class, "M2");
        viper  = hardwareMap.get(DcMotor.class, "M5");
        inTAKE = hardwareMap.get(CRServo.class, "S1");
        pulse = hardwareMap.get(Servo.class, "S2");
        //arm = hardwareMap.get(DcMotor.class, "M6");
        extensor = hardwareMap.get(DcMotor.class,"M7");

        /*gm.setPower(0.5);
        sleep(3000);
        gm.setPower(0);*/

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        pulse.setPosition(0);
        sleep(500);

        while(opModeIsActive()){
            garraComandos();

            viperComandos();

            movimentacao();

            /*telemetry.addData("Garra:",  gm.getCurrentPosition());
            telemetry.update();*/

        }

    }

    public void viperComandos() {

        if(viper.getCurrentPosition() < maxPositionViper) {
            if (gamepad2.right_trigger > 0) {
                viper.setPower(gamepad2.right_trigger * 0.8);
            } else if (gamepad2.left_trigger > 0) {
                viper.setPower(gamepad2.right_trigger * -0.8);

            }
        } else {
            viper.setPower(0);
        }

    }

    public void garraComandos(){
        // In Take - Rotor
        if(clockROTOR == 1){
            inTAKE.setPower(1);
            if(gamepad2.a){
                clockROTOR = 0;
                sleep(500);
            }
        } else if (clockROTOR == 0){
            inTAKE.setPower(0);
            if(gamepad2.a){
                clockROTOR = 1;
                sleep(500);
                }
        }else if (unclockROTOR == 1){
                inTAKE.setPower(-1);
                if(gamepad2.b){
                    clockROTOR = 0;
                    sleep(500);
                }
        } else if (unclockROTOR == 0){
                inTAKE.setPower(0);
                if(gamepad2.b){
                    clockROTOR = 1;
                    sleep(500);
                }
        }


        // Pulso
        if(gamepad2.dpad_up) {
            pulse.setPosition(1);

        } else if (gamepad2.dpad_down) {
            pulse.setPosition(0);
        }

        // Extensor
        if(extensor.getCurrentPosition() < maxPosition){
            extensor.setPower(0);
        } else {
            extensor.setPower(gamepad2.right_stick_y * -0.7);
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
