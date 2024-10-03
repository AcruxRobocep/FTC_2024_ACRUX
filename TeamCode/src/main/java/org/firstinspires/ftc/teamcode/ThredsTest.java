package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class ThredsTest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "M1");
        waitForStart();
        while(isStopRequested()){
           // sla();
            slo();
        }
        //slo();
    }

    void sla(){
        new Thread(){
            @Override
            public void run() {
                while(!isStopRequested()){
                   motor.setPower(1);
                }

            }
        }.start();
    }

    void slo(){
        new Thread(){
            @Override
            public void run() {
                while(!isStopRequested()){
                   telemetry.addData("LALAL:","");
                   telemetry.update();

                }

            }
        }.start();
    }
}
