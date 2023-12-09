package org.firstinspires.ftc.teamcode.drive;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends LinearOpMode {


    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;


    @Override
    public void runOpMode() {

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;


        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
            telemetry.update();
        }



        Side side = propPipeline.getLocation();
        portal.close();

        Pose2d yellowScorePos = new Pose2d();
        Pose2d purpleScorePos = new Pose2d();
        Pose2d parkPos = new Pose2d();


        // 0.3, 300

        switch (side) {
            case LEFT:
                // TODO: add poses
                yellowScorePos = new Pose2d(0,0,0);
                purpleScorePos = new Pose2d(0,0,0);
                parkPos = new Pose2d(0,0,0);
                break;
            case CENTER:
                yellowScorePos = new Pose2d(0,0,0);
                purpleScorePos = new Pose2d(0,0,0);
                parkPos = new Pose2d(0,0,0);
                break;
            case RIGHT:
                yellowScorePos = new Pose2d(0,0,0);
                purpleScorePos = new Pose2d(0,0,0);
                parkPos = new Pose2d(0,0,0);
                break;
            default:
                break;

        }
        waitForStart();


    }

}