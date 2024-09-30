package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous
public class teste_automato extends LinearOpMode {
    DistanceSensor sensorDist;

    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    DcMotor arm;
    DcMotor viper;

    Servo pulse;
    CRServo inTake;

    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        sensorDist = hardwareMap.get(DistanceSensor.class, "D1");

        RF  = hardwareMap.get(DcMotor.class, "M3");
        RB  = hardwareMap.get(DcMotor.class, "M1");
        LF  = hardwareMap.get(DcMotor.class, "M4");
        LB  = hardwareMap.get(DcMotor.class, "M2");
        imu = hardwareMap.get(IMU.class,"imu");


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        move_for_cm(100,0.5);
        move_for_cm(50,0.5,'D');
        move_for_cm(10, 0.5);






    }
    void move(double spd){
            RF.setPower(spd);
            RB.setPower(spd);
            LF.setPower(spd);
            LB.setPower(spd);
        }

    void move(double spd,char direction){
         if(direction == 'D'){
            RF.setPower(spd);
            RB.setPower(-spd);
            LF.setPower(-spd);
            LB.setPower(spd);
        } else if(direction == 'E'){
            RF.setPower(-spd);
            RB.setPower(spd);
            LF.setPower(spd);
            LB.setPower(-spd);
        } else {
            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
        }

    }

    void move_for_cm(int cm, double spd){
        while(sensorDist.getDistance(DistanceUnit.CM) < cm){
            move(spd);
        }
        move(0);
        sleep(500);
    }

    void move_for_cm(int cm, double spd, int time_ms){
        while(sensorDist.getDistance(DistanceUnit.CM) < cm){
            move(spd);
        }
        move(0);
        sleep(time_ms);
    }

    void move_for_cm(int cm, double spd, char direction){
        while(sensorDist.getDistance(DistanceUnit.CM) < cm){
            move(spd,direction);
        }
        move(0);
        sleep(500);
    }

   void move_for_cm(int cm, double spd, char direction, int time_ms){
        while(sensorDist.getDistance(DistanceUnit.CM) < cm){
            move(spd, direction);
        }
       move(0);
       sleep(time_ms);
    }





}
