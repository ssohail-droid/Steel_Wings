package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field-Oriented Mecanum Drive2.1", group="Iterative OpMode")
public class FieldOrientedMecanum2 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private GoBildaPinpointDriver odo;

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backRight = hardwareMap.get(DcMotor.class, "right_back");

        // Reverse right side motors to match direction
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, 0.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Update odometry to get the latest heading
        odo.update();
        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Read joystick inputs
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Strafe Left/Right
        double turn = gamepad1.right_stick_x; // Rotation

        // Apply field-oriented transformation
        double cosA = Math.cos(heading);
        double sinA = Math.sin(heading);
        double tempX = x * cosA - y * sinA;
        double tempY = x * sinA + y * cosA;

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
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Motor Power", "FL: %.2f, BL: %.2f, FR: %.2f, BR: %.2f",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        telemetry.update();
    }
}
