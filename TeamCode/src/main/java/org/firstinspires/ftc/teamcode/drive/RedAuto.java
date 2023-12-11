package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode {


    private FindRegionPipeline propPipeline;



    @Override
    public void runOpMode() throws InterruptedException {

        Servo leftWrist = hardwareMap.servo.get("leftWrist");
        Servo rightWrist = hardwareMap.servo.get("rightWrist");
        DcMotor flip = hardwareMap.dcMotor.get("flip");


        Servo leftClaw = hardwareMap.servo.get("leftClaw");
        Servo rightClaw = hardwareMap.servo.get("rightClaw");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.RED;


        propPipeline = new FindRegionPipeline(Globals.COLOR);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        camera.setPipeline(propPipeline);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
            telemetry.addData("left", propPipeline.getLeftAvgFinal());
            telemetry.addData("right", propPipeline.getRightAvgFinal());
            telemetry.update();
        }



        Side side = propPipeline.getLocation();

        Trajectory yellowScorePos = null;
        Trajectory purpleScorePos = null;
        Trajectory parkPos = null;


        // 0.3, 300
        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        switch (side) {
            case LEFT:
                // TODO: add poses
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(15, -38), Math.toRadians(180))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(51,-45), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineTo(new Vector2d(60,-60), 0)
                        .build();
                telemetry.addData("POS", "LEFT");
                break;
            case CENTER:
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(12, -38), Math.toRadians(90))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(15, -45 ),Math.toRadians(90))
                        .splineTo(new Vector2d(51,-32), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineTo(new Vector2d(60,-60), 0)
                        .build();
                telemetry.addData("POS", "CENTER");
                break;
            case RIGHT:
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(33, -56), 0)
                        .splineTo(new Vector2d(38, -36), Math.toRadians(180))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(51,-40), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineTo(new Vector2d(60,-60), 0)
                        .build();
                telemetry.addData("POS", "RIGHT");
                break;
            default:
                break;
        }
        telemetry.update();
        rightClaw.setPosition(0);
        leftClaw.setPosition(1);
        waitForStart();

        int startPosition = flip.getCurrentPosition();
        flip.setTargetPosition(-400);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(0.8);



        drive.followTrajectory(purpleScorePos);
        flip.setTargetPosition(startPosition);
        rightWrist.setPosition(0.6);
        leftWrist.setPosition(0.4);
        sleep(500);
        leftClaw.setPosition(0);
        sleep(1000);
        rightWrist.setPosition(0.8);
        leftWrist.setPosition(0.2);
        flip.setTargetPosition(-1587);

//        drive.followTrajectory(yellowScorePos);
//        while (!isStopRequested() && Math.abs(flip.getTargetPosition()- flip.getCurrentPosition()) > 100) {
//
//        }
//        sleep(500);
//        rightClaw.setPosition(1);
//        sleep(3000);
        drive.followTrajectory(parkPos);


    }



}