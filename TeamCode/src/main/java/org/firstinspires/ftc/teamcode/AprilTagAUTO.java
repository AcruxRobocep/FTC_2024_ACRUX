package org.firstinspires.ftc.teamcode;

import static java.nio.file.Files.move;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
public class AprilTagTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;


    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    DcMotor viper;

    Servo pulse;
    Servo outTake;
    CRServo inTake;

    IMU imu;
    NormalizedColorSensor SC;

    double dist = 1000;
    double Aprilyaw = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        initDevices();
        initAprilTag();
        sleep(500);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        imu.resetYaw();
        waitForStart();

        outTake.setPosition(1);
        //Thread AprilTagData = new Thread(new tarefa1());
        //AprilTagData.start();

        while (!(Aprilyaw > -2 && Aprilyaw < 2)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    Aprilyaw = detection.ftcPose.bearing;

                    telemetry.addData("alinhamento: ", Aprilyaw);
                    telemetry.addData("dist: ", dist);

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                telemetry.update();
            }
            move(0.3, 'D');
        }
        move(0);
        sleep(1000);
        while (dist > 80) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    dist = detection.ftcPose.range;
                    telemetry.addData("alinhamento: ", Aprilyaw);
                    telemetry.addData("dist: ", dist);

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }

                telemetry.update();

            }
            move(0.4);
        }
        move(0);

        YawPitchRollAngles giro1 = imu.getRobotYawPitchRollAngles();
        double YAw = giro1.getYaw();
        imu.resetYaw();
        while (YAw < 30){
            giro1 = imu.getRobotYawPitchRollAngles();
            YAw = giro1.getYaw();
            muveRD(0.4);
            telemetry.addData("Yaw", YAw);
            telemetry.update();
        }
        move(0);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setTargetPosition(0);
        RB.setTargetPosition(567);
        LF.setTargetPosition(487);
        LB.setTargetPosition(514);
        viper.setTargetPosition(-3165);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viper.setPower(0.8);
        move(0.3);

        sleep(2000);
        outTake.setPosition(0);
        sleep(500);
        outTake.setPosition(1);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setTargetPosition(1);
        viper.setPower(0.8);
        viper.setPower(0);
        sleep(500);
        RF.setTargetPosition(0);
        RB.setTargetPosition(0);
        LF.setTargetPosition(0);
        LB.setTargetPosition(0);
        viper.setTargetPosition(0);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        viper.setPower(0.8);

        sleep(2000);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (YAw > 11){
            giro1 = imu.getRobotYawPitchRollAngles();
            YAw = giro1.getYaw();
            muveLD(0.4);
            telemetry.addData("Yaw", YAw);
            telemetry.update();
        }
        move(0);

        while(opModeIsActive()){
            telemetry.addData("RF", RF.getCurrentPosition());
            telemetry.addData("RB", RB.getCurrentPosition());
            telemetry.addData("LF", LF.getCurrentPosition());
            telemetry.addData("LB", LB.getCurrentPosition());
            telemetry.addData("Viper", viper.getCurrentPosition());
            telemetry.update();

        }
    }


    public void initAprilTag(){
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


    public void initDevices() {

        RF = hardwareMap.get(DcMotor.class, "M3");
        RB = hardwareMap.get(DcMotor.class, "M1");
        LF = hardwareMap.get(DcMotor.class, "M4");
        LB = hardwareMap.get(DcMotor.class, "M2");
        viper = hardwareMap.get(DcMotor.class,"M5");
        outTake = hardwareMap.get(Servo.class, "S3");

        /*pulse = hardwareMap.get(Servo.class,"S1");
        inTake = hardwareMap.get(CRServo.class,"S2");
        */

        inTake = hardwareMap.get(CRServo.class, "S1");

        imu = hardwareMap.get(IMU.class, "imu");
        //sc = hardwareMap.get()  SENSOR DE CORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR!!!!!ESTÁ AQUIIIIIII!!!
    }

    public void move(double spd) {
        RF.setPower(spd);
        RB.setPower(spd);
        LF.setPower(spd);
        LB.setPower(spd);
    }

    public void muveRD(double spd){
        RF.setPower(spd);
        RB.setPower(spd);
        LF.setPower(-spd);
        LB.setPower(-spd);
    }

    public void muveLD(double spd){
        RF.setPower(-spd);
        RB.setPower(-spd);
        LF.setPower(spd);
        LB.setPower(spd);
    }

    public void move(double spd, char direction) {
        if (direction == 'D') {
            RF.setPower(-spd);
            RB.setPower(spd);
            LF.setPower(spd);
            LB.setPower(-spd);
        }
        else if (direction == 'E') {
            RF.setPower(spd);
            RB.setPower(-spd);
            LF.setPower(-spd);
            LB.setPower(spd);
        }
        else {
            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
        }
    }
}



