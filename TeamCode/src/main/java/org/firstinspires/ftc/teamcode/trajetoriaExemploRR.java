package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class trajetoriaExemploRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d posInicial = new Pose2d(10,-8,Math.toRadians(90));

        // trajetorias de movimento
        Trajectory trajetoria = drive.trajectoryBuilder(new Pose2d())
        .strafeRight(10).build();
        waitForStart();

        Trajectory trajetoria2 = drive.trajectoryBuilder((trajetoria.end()))
        .forward(10).build();

        if(isStopRequested()) return;
        drive.followTrajectory(trajetoria);
        drive.turn(Math.toRadians(90)); // Gira 90 graus em seu eixo
        drive.followTrajectory(trajetoria2);

    }
}
