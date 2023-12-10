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
@Autonomous(name = "Blue Auto")
public class BlueAuto extends LinearOpMode {


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
        Globals.COLOR = Side.BLUE;


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
        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        switch (side) {
            case LEFT:
                // TODO: add poses
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(33, 56), 0)
                        .splineTo(new Vector2d(42, 33), Math.toRadians(180))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(47,40), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(yellowScorePos.end())
                        .splineTo(new Vector2d(60,60), 0)
                        .build();
                telemetry.addData("POS", "LEFT");
                break;
            case CENTER:
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(33, 56), 0)
                        .splineTo(new Vector2d(42, 33), Math.toRadians(180))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(47,40), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(yellowScorePos.end())
                        .splineTo(new Vector2d(60,60), 0)
                        .build();
                telemetry.addData("POS", "CENTER");
                break;
            case RIGHT:
                purpleScorePos = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(33, 56), 0)
                        .splineTo(new Vector2d(42, 33), Math.toRadians(180))
                        .build();

                yellowScorePos = drive.trajectoryBuilder(purpleScorePos.end())
                        .splineToConstantHeading(new Vector2d(47,40), Math.toRadians(180))
                        .build();
                parkPos = drive.trajectoryBuilder(yellowScorePos.end())
                        .splineTo(new Vector2d(60,60), 0)
                        .build();
                telemetry.addData("POS", "RIGHT");
                break;
            default:
                break;
        }
        telemetry.update();
        waitForStart();

        int startPosition = flip.getCurrentPosition();
        flip.setTargetPosition(startPosition);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPower(1);
        rightClaw.setPosition(0);
        leftClaw.setPosition(1);


        drive.followTrajectory(purpleScorePos);
        rightWrist.setPosition(0.6);
        leftWrist.setPosition(0.4);
        rightClaw.setPosition(1);
        sleep(3000);
        drive.followTrajectory(yellowScorePos);
        rightWrist.setPosition(0.75);
        leftWrist.setPosition(0.25);
        flip.setTargetPosition(-1700);
        sleep(3000);
        leftClaw.setPosition(0);
        sleep(3000);
        drive.followTrajectory(parkPos);


    }



}