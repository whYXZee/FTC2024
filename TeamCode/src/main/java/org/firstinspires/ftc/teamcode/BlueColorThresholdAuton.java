package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous

public class BlueColorThresholdAuton extends LinearOpMode {
    public int position;
    private OpenCvCamera webcam;

    private DcMotor intakeMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;

    double turnDistance;
    double ticksPerInch;
    double turnRadius;
    int velocity;
    double rotationAdjust;

    private static final int CAMERA_WIDTH  = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution

    private double CrLowerUpdate = 150;
    private double CbLowerUpdate = 50;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(100, 150, 50);
    public static Scalar scalarUpperYCrCb = new Scalar(200, 255, 255);

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        position = 0;
        int wheelDiameter;
        int motorTicksPerRev;
        int gearRatio;
        double W;
        double L;

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Reset the encoder during the init phase
        Stop_and_Reset_Motor_Encoders();
        // Reverse the right side motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        // Enter the wheel diameter of your robot in MM
        wheelDiameter = 96;
        // Enter the desired speed of the robot in in/sec
        velocity = 30;
        // Enter the motor encoder ticks per rev
        motorTicksPerRev = 28;
        // Enter the gear ratio for your drive motors
        gearRatio = 12;
        // Enter the width of your robots wheels in inches
        W = 18;
        // Enter the length of your robots wheels in inches
        L = 17.75;
        // Create a variable to adjust rotation
        rotationAdjust = 1.35;
        // Calculate the Ticks per Inch of your robot
        ticksPerInch = motorTicksPerRev * gearRatio * (1 / (2 * Math.PI * (wheelDiameter / 2))) * (25.4 / 1);
        turnRadius = Math.sqrt((L / 2) * (L / 2) + (W / 2) * (W / 2));
        telemetry.addData("ticksPerInch", ticksPerInch);
        telemetry.addData("turnRadius", turnRadius);
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            sleep(5000);
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            telemetry.addData("Box Midpoint", myPipeline.getRectMidpointX());
            // telemetry.update();
            sleep(5000);

            if (myPipeline.getRectArea() > 2) {
                if (myPipeline.getRectMidpointX() > 1000) {
                    AUTONOMOUS_A();
                } else if (myPipeline.getRectMidpointX() > 300) {
                    AUTONOMOUS_B();
                } else {
                    AUTONOMOUS_C();
                }
            }


            /* if position 1
            if (position == 1) {
                telemetry.addLine("Position 1");
                telemetry.update();
                runToPosition(1500, 1500, 1500, 1500, 0.4);
                runToPosition(-300, -300, -300, -300, 0.4);
                intakeMotor.setPower(-1);
                sleep(1800);
            }
            // if position 3
            else if (position == 3) {
                telemetry.addLine("Position 3");
                telemetry.update();
                runToPosition(700, 700, 700, 700, 0.4);
                runToPosition(-100, 100, -100, 700, 0.4);
                //runToPosition(-300, -300, -300, -300, 0.4);
                //intakeMotor.setPower(-1);
                sleep(1800);

            } else if (position == 2){
                telemetry.addLine("Position 2");
                telemetry.update();
                runToPosition(1100, 1100, 1100, 1100, 0.4);
                intakeMotor.setPower(-1);
                runToPosition(100, 100, 100, 100, 0.4);
                intakeMotor.setPower(-1);
                sleep(1800);
            }

             */
        }

    }
    void runToPosition(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos, double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(-frontLeftPos);
        frontRightMotor.setTargetPosition(-frontRightPos);
        backLeftMotor.setTargetPosition(-backLeftPos);
        backRightMotor.setTargetPosition(-backRightPos);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        while(frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {}
    }
    public void testing(ContourPipeline myPipeline) {
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max) {
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void AUTONOMOUS_A() {
        telemetry.addLine("Autonomous A");
        position = 1;
    }
    public void AUTONOMOUS_B() {
        telemetry.addLine("Autonomous B");
        position = 2;
    }
    public void AUTONOMOUS_C() {
        telemetry.addLine("Autonomous C");
        position = 3;
    }
    private void Change_RunMode_to_Run_to_Position() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void Drive_Robot_in__Y_Direction(int yPos) {
        Stop_and_Reset_Motor_Encoders();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (yPos * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (yPos * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (yPos * ticksPerInch));
        backRightMotor.setTargetPosition((int) (yPos * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (backLeftMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Moving in +Y direction, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void Report_Current_Position_of_Motors() {
        telemetry.addData("frontLeft Position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("backtLeft Position", backLeftMotor.getCurrentPosition());
        telemetry.addData("frontRight Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("backRight Position", backRightMotor.getCurrentPosition());
    }

    /**
     * Describe this function...
     */
    private void Drive_Robot_in__X_Direction(int xPos) {
        Stop_and_Reset_Motor_Encoders();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (xPos * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (-xPos * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (-xPos * ticksPerInch));
        backRightMotor.setTargetPosition((int) (xPos * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (backLeftMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Moving in +X direction, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void Rotate_robot__rz_direction2(int _rz) {
        Stop_and_Reset_Motor_Encoders();
        // Calculate the rotation distance
        turnDistance = 2 * Math.PI * turnRadius * (_rz / 360);
        telemetry.addData("turnDistance", turnDistance);
        telemetry.update();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (-rotationAdjust * turnDistance * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (-rotationAdjust * turnDistance * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (rotationAdjust * turnDistance * ticksPerInch));
        backRightMotor.setTargetPosition((int) (rotationAdjust * turnDistance * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (frontRightMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Rotating the robot in +rz, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void Stop_and_Reset_Motor_Encoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Drive_Robot_in__Y_Direction2(int yPos) {
        Stop_and_Reset_Motor_Encoders();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (-yPos * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (-yPos * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (-yPos * ticksPerInch));
        backRightMotor.setTargetPosition((int) (-yPos * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (backLeftMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Moving in -Y direction, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_Robot_in__X_Direction2(double xPos) {
        Stop_and_Reset_Motor_Encoders();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (-xPos * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (xPos * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (xPos * ticksPerInch));
        backRightMotor.setTargetPosition((int) (-xPos * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (backLeftMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Moving in -X direction, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void Rotate_robot__rz_direction(int _rz2) {
        Stop_and_Reset_Motor_Encoders();
        // Calculate the rotation distance
        turnDistance = 2 * Math.PI * turnRadius * (_rz2 / 360);
        telemetry.addData("turnDistance", turnDistance);
        telemetry.update();
        // Set the Motors target position
        frontLeftMotor.setTargetPosition((int) (rotationAdjust * turnDistance * ticksPerInch));
        backLeftMotor.setTargetPosition((int) (rotationAdjust * turnDistance * ticksPerInch));
        frontRightMotor.setTargetPosition((int) (-rotationAdjust * turnDistance * ticksPerInch));
        backRightMotor.setTargetPosition((int) (-rotationAdjust * turnDistance * ticksPerInch));
        // Switch to RUN_TO_POSITION mode
        Change_RunMode_to_Run_to_Position();
        // Start the motor moving by setting the max velocity
        ((DcMotorEx) frontLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backLeftMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) frontRightMotor).setVelocity(velocity * ticksPerInch);
        ((DcMotorEx) backRightMotor).setVelocity(velocity * ticksPerInch);
        while (backLeftMotor.isBusy() && !isStopRequested()) {
            telemetry.addData("Status:", "Rotating the robot in -rz, Waiting for the robot to reach its target position");
            Report_Current_Position_of_Motors();
            telemetry.update();
        }
    }
}


