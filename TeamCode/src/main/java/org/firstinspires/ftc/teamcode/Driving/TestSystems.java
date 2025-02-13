package org.firstinspires.ftc.teamcode.Driving;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.FloorGrippySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.GrippySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.PickupSubsystem;

@TeleOp
public class TestSystems extends LinearOpMode {
    // Variables
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo slideServo;
    private Servo floorServo;
    private IMU imu;
    private double targetSlidePosition;
    private double targetFloorSlidePosition;
    private double fieldOffset;

    @Override
    public void runOpMode() {
        final DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        final DcMotor rearRight = hardwareMap.dcMotor.get("rearRight");
        final DcMotor rearLeft = hardwareMap.dcMotor.get("rearLeft");
        final DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        final Servo slideServo = hardwareMap.get(Servo.class, "slideServo");
        final Servo floorServo = hardwareMap.get(Servo.class, "floorServo");
        final DcMotorEx floorSlide = hardwareMap.get(DcMotorEx.class, "floorSlide");
        final DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        GrippySubsystem grippySubsystem = new GrippySubsystem(slideServo);
        FloorGrippySubsystem floorGrippySubsystem = new FloorGrippySubsystem(floorServo);
        SlideSubsystem slideSubsystem = new SlideSubsystem(slideMotor);
        PickupSubsystem pickupSubsystem = new PickupSubsystem(floorSlide);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            boolean scu = gamepad1.right_bumper;
            boolean scd = gamepad1.left_bumper;

            if (gamepad1.right_stick_button) {fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);}

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            double headingRad = Math.toRadians(botHeading);

            {
                double temp = y * Math.cos(headingRad) + x * Math.sin(headingRad);
                x = -y * Math.sin(headingRad) + x * Math.cos(headingRad);
                y = temp;
            }

            double turningSpeed = 0.3;

            if (Math.abs(rx) > 0.1) {
                rx = turningSpeed * Math.signum(rx);
            } else {
                rx = 0;
            }

            if (scu) {targetSlidePosition += 1;}
            if (scd) {targetSlidePosition -= 1;}

            if (gamepad1.right_trigger > 0.1) {targetFloorSlidePosition += 1;}
            if (gamepad1.left_trigger > 0.1) {targetFloorSlidePosition -= 1;}

            // Main logic
            if (gamepad1.a) {
                grippySubsystem.setServoPosition(0);
            } else if (gamepad1.b) {
                grippySubsystem.setServoPosition(-45);
            } else if (gamepad1.left_stick_button) {grippySubsystem.setServoPosition(0);}

            if (gamepad1.x) {
                floorGrippySubsystem.setServoPosition(0);
            } else if (gamepad1.y) {
                floorGrippySubsystem.setServoPosition(-45);
            } else if (gamepad1.left_stick_button) {grippySubsystem.setServoPosition(0);}

            if (slideSubsystem.getSlideAmps() > 5) {slideSubsystem.setSlidePower(0);}

            if (pickupSubsystem.getSlideAmps() > 5) {pickupSubsystem.setFloorSlidePower(0);}

            slideSubsystem.setTargetPosition(targetSlidePosition);
            pickupSubsystem.setTargetPosition(targetFloorSlidePosition);
            pickupSubsystem.update();
            slideSubsystem.update();

            // Power stuff
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);

            // Telemetry
            String emptyVariable = " ";
            telemetry.addData("specimen slide target", slideSubsystem.getTargetPosition());
            telemetry.addData("specimen slide actual", slideSubsystem.getCurrentPosition());
            telemetry.addData("", emptyVariable);
            telemetry.addData("floor slide target", pickupSubsystem.getTargetPosition());
            telemetry.addData("floor slide actual", pickupSubsystem.getCurrentPosition());
            telemetry.addData("", emptyVariable);
            telemetry.addData("specimen slide amp draw", slideSubsystem.getSlideAmps());
            telemetry.addData("floor slide amp draw", pickupSubsystem.getSlideAmps());
            telemetry.addData("", emptyVariable);
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}
