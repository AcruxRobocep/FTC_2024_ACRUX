package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class garra_planoB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo inTAKE;

        Servo pulse;

        DcMotor arm;
        DcMotor extensor;

        int maxPosition = -1000;
        int stateROTOR = 1;

        inTAKE = hardwareMap.get(CRServo.class, "S1");
        pulse = hardwareMap.get(Servo.class, "S2");
        arm = hardwareMap.get(DcMotor.class, "M5");
        extensor = hardwareMap.get(DcMotor.class,"M6");

        pulse.setPosition(0);
        extensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(opModeIsActive()){

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
    }
}
