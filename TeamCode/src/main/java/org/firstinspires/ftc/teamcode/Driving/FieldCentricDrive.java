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
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.GrippySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SlideSubsystem;

@TeleOp
public class FieldCentricDrive extends LinearOpMode {
    // Variables
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private IMU imu;
    private double targetServoPosition;
    private double targetSlidePosition;
    private double targetArmPosition;
    private double fieldOffset;
    private int presetCycle = 0;
    private double lastPresetCycle = 0;
    private boolean lastLeftBumperState = false;
    private boolean lastRightBumperState = false;
    private boolean isArmBeingControlled = false;
    private boolean isArmPastHS;

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
        final Servo grippyServo = hardwareMap.servo.get("grippy");
        final DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        final IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        GrippySubsystem grippySubsystem = new GrippySubsystem(grippyServo);
        SlideSubsystem slideSubsystem = new SlideSubsystem(slideMotor);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double ac = gamepad1.right_stick_y;
            double sR = gamepad1.left_trigger;
            double sF = gamepad1.right_trigger;

            if (gamepad1.y) {
                fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

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

            boolean isArmBeingControlled = (Math.abs(ac) > 0.1);

            // Presets
            if (presetCycle != lastPresetCycle && !isArmBeingControlled) {
                switch (presetCycle) {
                    case 1:
                        targetArmPosition = 0;
                        break;
                    case 2:
                        targetArmPosition = -775;
                        break;
                    case 3:
                        targetArmPosition = -1815;
                        break;
                    case 4:
                        targetArmPosition = -3350;
                        break;
                    default:
                        telemetry.addData("preset", "none selected");
                        telemetry.update();
                        break;
                }
                armSubsystem.setPosition(targetArmPosition);
                lastPresetCycle = presetCycle;
            }

            // Main logic
            if (gamepad1.a) {
                targetServoPosition = 90;
                grippySubsystem.setServoPosition(targetServoPosition);
            } else if (gamepad1.b) {
                targetServoPosition = -90;
                grippySubsystem.setServoPosition(targetServoPosition);
            }

            // Main logic for slide
            isArmPastHS = slideSubsystem.getCurrentPosition() > 500;
            if (gamepad1.dpad_down && !isArmPastHS) {targetSlidePosition = 3;}
            if (gamepad1.dpad_up && !isArmPastHS) {targetSlidePosition = 3800;}
            if (gamepad1.dpad_left && !isArmPastHS) {targetSlidePosition = 1500;}
            if (gamepad1.dpad_right && !isArmPastHS) {targetSlidePosition = 500;}
            // Logic to save the slide from cooking itself
            if (targetSlidePosition < 3) {targetSlidePosition = 4;}
            if (targetSlidePosition > 3800) {targetSlidePosition = 3790;}
            if (slideSubsystem.getSlideAmps() > 5) {slideSubsystem.setSlidePower(0);}
            if (isArmPastHS) {slideSubsystem.setTargetPosition(3);}
            // Logic to save arm from cooking itself
            if (targetArmPosition < -775) {targetArmPosition = -765;}
            if (targetArmPosition > -3350) {targetArmPosition = -3340;}
//            if (armSubsystem.getArmAmps() > 5) {armSubsystem.setArmPower(0);} not implemented yet
            slideSubsystem.setTargetPosition(targetSlidePosition);

            // Power stuff
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x + rx) / denominator;
            double rearRightPower = (y - x + rx) / denominator;
            double rearLeftPower = (y + x - rx) / denominator;
            double frontLeftPower = (y - x - rx) / denominator;
            double slidePower = (sF + sR) / 2 / denominator;

            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
            rearLeft.setPower(rearLeftPower);
            frontLeft.setPower(frontLeftPower);
            slideMotor.setPower(slidePower);
        }
    }
}
