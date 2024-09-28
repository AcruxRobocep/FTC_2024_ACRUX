package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class garra_planoA extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo pincaA;
        Servo pincaB;
        Servo pulso;
        int count_pulso = 1 ;

        pincaA = hardwareMap.get(Servo.class,"S1");
        pincaB = hardwareMap.get(Servo.class, "S2");
        pulso = hardwareMap.get(Servo.class,"S3");

        waitForStart();
        pulso.setPosition(0);
        //pulso.scaleRange(); pincaA.scaleRange(); pincaB.scaleRange();
        // pulso.setDirection();


        while(opModeIsActive()){
            if(gamepad2.a){

            }
        }
    }
}
