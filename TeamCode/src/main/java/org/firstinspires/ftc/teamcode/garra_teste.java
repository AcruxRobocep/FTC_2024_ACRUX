package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class garra_teste extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        Servo p1;
        Servo p2;
        Servo pulse;
        boolean get;
        get = false;


        p1 = hardwareMap.get(Servo.class, "S1");
        p2 = hardwareMap.get(Servo.class,"S2");
        pulse = hardwareMap.get(Servo.class,"S3");

        p1.setPosition(0);
        p2.setDirection(Servo.Direction.REVERSE);
        p2.setPosition(0);

        pulse.setPosition(0);
        waitForStart();

        while(!isStopRequested()) {

            if(gamepad1.dpad_up){
                pulse.setPosition(1);
            }
            else if(gamepad1.dpad_down) {
                pulse.setPosition(0);
            }

            if(!get){
                if(gamepad1.a) {
                    get = true;
                    p1.setPosition(1);
                    p2.setPosition(1);
                    sleep(500);
                }
            } else {
                if(gamepad1.a){
                    get = false;
                    p1.setPosition(0);
                    p2.setPosition(0);
                    sleep(500);

                }
            }





        }
    }
}
