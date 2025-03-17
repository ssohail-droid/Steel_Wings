package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Field Oriented Mecanum", group="Iterative OpMode")
public class FieldOrientedMecanum extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    private GoBildaPinpointDriver odo; // Odometry sensor

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize goBILDA Pinpoint Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU(); // Reset IMU and position

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Get heading from odometry (IMU)
        odo.update(); // Update odometry readings
        double robotAngle = odo.getPosition().getHeading(AngleUnit.RADIANS); // Heading in radians

        // Controller input
        double y = -gamepad1.left_stick_y; // Forward
        double x = gamepad1.left_stick_x;  // Strafe
        double turn = gamepad1.right_stick_x; // Rotation

        // Field-oriented transformation
        double tempX = x * Math.cos(robotAngle) - y * Math.sin(robotAngle);
        double tempY = x * Math.sin(robotAngle) + y * Math.cos(robotAngle);

        // Mecanum drive calculations
        double denominator = Math.max(Math.abs(tempY) + Math.abs(tempX) + Math.abs(turn), 1);
        double frontLeftPower = (tempY + tempX + turn) / denominator;
        double backLeftPower = (tempY - tempX + turn) / denominator;
        double frontRightPower = (tempY - tempX - turn) / denominator;
        double backRightPower = (tempY + tempX - turn) / denominator;

        // Set motor power
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotAngle));
        telemetry.addData("Motor Power", "FL (%.2f) BL (%.2f) FR (%.2f) BR (%.2f)",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
