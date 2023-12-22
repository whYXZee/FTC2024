package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class FieldCentricPID extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degree = ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * 28);

    private DcMotorEx leftArm;
    private DcMotorEx rightArm;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftArm = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "leftArm");
        rightArm = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "rightArm");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int rightArmPos = rightArm.getCurrentPosition();
        double pidRight = controller.calculate(rightArmPos, target);
        double ffRight = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double powerRight = pidRight * ffRight;

        int leftArmPos = leftArm.getCurrentPosition();
        double pidLeft = controller.calculate(leftArmPos, target);
        double ffLeft = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double powerLeft = pidLeft * ffLeft;

        rightArm.setPower(-powerRight);
        leftArm.setPower(powerLeft);

        telemetry.addData("Left Arm Position", leftArmPos);
        telemetry.addData("Right Arm Position", rightArmPos);
        telemetry.addData("Target", target);
    }
}