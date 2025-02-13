package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class robo_teste extends LinearOpMode {

    DcMotor ML;
    DcMotor MR;
    DcMotor braco;
    DcMotor braco_extensao;

    Servo garra;
    boolean statusGarra = false;



    @Override
    public void runOpMode() throws InterruptedException {
        ML = hardwareMap.get(DcMotor.class,"M1");
        MR = hardwareMap.get(DcMotor.class,"M2");
        braco  = hardwareMap.get(DcMotor.class,"M3");
        braco_extensao = hardwareMap.get(DcMotor.class,"M4");
        garra = hardwareMap.get(Servo.class,"S1");

        ML.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Servo:",garra.getPosition());
            telemetry.update();
            ML.setPower(gamepad1.left_stick_y);
            MR.setPower(gamepad1.right_stick_y);

            if(gamepad2.a){
                garra.setPosition(0.6);
            }else if(gamepad2.b){
                garra.setPosition(0.4 );
            }

            if(gamepad2.left_trigger > 0){
                braco.setPower(0.5*gamepad2.left_trigger);
            }else if(gamepad2.right_trigger > 0){
                braco.setPower(-0.5 * gamepad2.right_trigger);
            }else {
                braco.setPower(0);
            }

            if(gamepad2.left_bumper){
                braco_extensao.setPower(0.5);
            }else if(gamepad2.right_bumper){
                braco_extensao.setPower(-0.5);
            }else {
                braco_extensao.setPower(0);
            }




        }
    }
}
