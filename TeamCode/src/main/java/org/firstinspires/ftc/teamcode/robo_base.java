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
    Servo outTake;
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
        outTake = hardwareMap.get(Servo.class, "S3");
        //arm = hardwareMap.get(DcMotor.class, "M6");
        extensor = hardwareMap.get(DcMotor.class,"M6");

        /*gm.setPower(0.5);
        sleep(3000);
        gm.setPower(0);*/

        //viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        outTake.setPosition(0);
        pulse.setPosition(0);
        sleep(500);

        while(opModeIsActive()){
            viperComandos();
            garraComandos();
            movimentacao();

            /*telemetry.addData("Garra:",  gm.getCurrentPosition());
            telemetry.update();*/

        }

    }

    public void viperComandos() {





            viper.setPower(gamepad2.left_stick_y * 0.8);

    }

    public void garraComandos(){
        // In Take - Rotor
        if(gamepad2.right_bumper){
            inTAKE.setPower(1);
        }
        else if (gamepad2.left_bumper){
            inTAKE.setPower(-1);
        }
        else {
            inTAKE.setPower(0);
        }


        // Pulso
        if(gamepad2.dpad_down) {
            pulse.setPosition(1);

        } else if (gamepad2.dpad_up) {
            pulse.setPosition(0);
        }

        // Extensor

        extensor.setPower(gamepad2.right_stick_y * 0.7);


        //Out Take
        if(gamepad2.x){
            outTake.setPosition(1);
        } else if (gamepad2.y) {
            outTake.setPosition(0);
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
