package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
        DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
        Servo leftWrist = hardwareMap.servo.get("leftWrist");
        Servo rightWrist = hardwareMap.servo.get("rightWrist");
        DcMotor flip = hardwareMap.dcMotor.get("flip");


        Servo leftClaw = hardwareMap.servo.get("leftClaw");
        Servo rightClaw = hardwareMap.servo.get("rightClaw");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        int startPosition = flip.getCurrentPosition();
        flip.setTargetPosition(startPosition);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double flipPower = 1f;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            rightElevator.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            leftElevator.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            if (gamepad1.right_bumper) {
                flip.setTargetPosition((flip.getTargetPosition())+20);
            } else if (gamepad1.left_bumper) {
                flip.setTargetPosition((flip.getTargetPosition())-20);
            }
            if (gamepad1.dpad_right) {
                rightWrist.setPosition(rightWrist.getPosition()+0.05);
                leftWrist.setPosition(1-rightWrist.getPosition());
            } else if (gamepad1.dpad_left) {
                rightWrist.setPosition(rightWrist.getPosition()-0.05);
                leftWrist.setPosition(1-rightWrist.getPosition());
            }
            if (gamepad1.a) {
                rightClaw.setPosition(1);
            }
            if (gamepad1.b) {
                leftClaw.setPosition(0.2);
            }
            if (gamepad1.y) {
                rightClaw.setPosition(0);
            }
            if (gamepad1.x) {
                leftClaw.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                flip.setTargetPosition(startPosition);
            }

            flip.setPower(flipPower);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("front left power", frontLeftPower);
            telemetry.addData("back left power", backLeftPower);
            telemetry.addData("front right power", frontRightPower);
            telemetry.addData("back right power", backRightPower);
            telemetry.addData("right elevator power", rightElevator.getPower());
            telemetry.addData("left elevator power", leftElevator.getPower());
            telemetry.addData("flip position", flip.getCurrentPosition()-startPosition);
            telemetry.addData("flip start position", startPosition);
            telemetry.addData("non-compensated flip position", flip.getCurrentPosition());
            telemetry.addData("flip target position", flip.getTargetPosition());
            telemetry.addData("right wrist position", rightWrist.getPosition());
            telemetry.addData("left wrist position", leftWrist.getPosition());
            telemetry.addData("right claw position", rightClaw.getPosition());
            telemetry.addData("left claw position", leftClaw.getPosition());

            telemetry.addData("bot heading", botHeading);
            telemetry.update();
        }
    }
}