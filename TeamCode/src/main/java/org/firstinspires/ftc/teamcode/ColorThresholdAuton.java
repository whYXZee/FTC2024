package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous

public class ColorThresholdAuton extends LinearOpMode {
    private int position;
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        // color thresholding code here
        // return int of prop position

        // if position 1
        if (position == 1) {
            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(-35.00, 60.00, Math.toRadians(-90.00)))
                    .lineToSplineHeading(new Pose2d(-35.00, 34.45, Math.toRadians(180.00)))
                    .build();

            drive.followTrajectory(myTrajectory);
        }
        // if position 3
         else if (position == 3) {
            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(-35.00, 60.00, Math.toRadians(-90.00)))
                    .lineToSplineHeading(new Pose2d(-35.00, 34.45, Math.toRadians(0.00)))
                    .build();

            drive.followTrajectory(myTrajectory);
        } else {
            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(-35.00, 60.00, Math.toRadians(-90.00)))
                    .lineToSplineHeading(new Pose2d(-35.00, 34.45, Math.toRadians(90.00)))
                    .build();

            drive.followTrajectory(myTrajectory);
        }



    }
}
