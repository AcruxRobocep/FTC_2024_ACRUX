package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class Autonomo_AprilTags extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;


    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    DcMotor viper;


    Servo pulse;
    CRServo inTake;


    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        initDevices();
        initAprilTag();
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        imu.resetYaw();
        waitForStart();
        readerAprilTag();
        sla();





    }

    void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }


    void initDevices() {

        RF = hardwareMap.get(DcMotor.class, "M3");
        RB = hardwareMap.get(DcMotor.class, "M1");
        LF = hardwareMap.get(DcMotor.class, "M4");
        LB = hardwareMap.get(DcMotor.class, "M2");
        /*
        viper = hardwareMap.get(DcMotor.class,"M5");
        pulse = hardwareMap.get(Servo.class,"S1");
        inTake = hardwareMap.get(CRServo.class,"S2");
        */

        imu = hardwareMap.get(IMU.class, "imu");


    }

    void move(double spd) {
        RF.setPower(spd);
        RB.setPower(spd);
        LF.setPower(spd);
        LB.setPower(spd);
    }

    void move(double spd, char direction) {
        if (direction == 'D') {
            RF.setPower(-spd);
            RB.setPower(spd);
            LF.setPower(spd);
            LB.setPower(-spd);
        } else if (direction == 'E') {
            RF.setPower(spd);
            RB.setPower(-spd);
            LF.setPower(-spd);
            LB.setPower(spd);
        } else {
            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
        }


    }


    void readerAprilTag() {
        new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            telemetry.addData("Distancia => ", detection.ftcPose.range);
                            telemetry.addData("Guinada   => ", detection.ftcPose.yaw);
                            telemetry.addData("Sla =>", detection.ftcPose.bearing);

                        } else {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                        }

                        telemetry.update();
                        }
                    }
                }
            }.start();
        }




        void sla(){
            new Thread(){
                @Override
                public void run() {
                    while(opModeIsActive()){
                        move(0.4);
                    }

                }
            }.start();
        }
    }



