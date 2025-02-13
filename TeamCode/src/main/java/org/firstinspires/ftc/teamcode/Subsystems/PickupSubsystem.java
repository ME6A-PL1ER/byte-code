package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PickupSubsystem extends SubsystemBase {

    private DcMotorEx floorSlide;
    private double targetPosition;
    private double currentPosition;
    private final ElapsedTime runtime = new ElapsedTime();

    private double previousError = 0, integral = 0;

    public PickupSubsystem(DcMotorEx slide) {
        this.floorSlide = floorSlide;
        floorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.targetPosition = 0;
        this.currentPosition = 0;
    }

    public PickupSubsystem() {
    }


    public void resetEncoder() {
        floorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        floorSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        currentPosition = floorSlide.getCurrentPosition();
        targetPosition = currentPosition;
    }

    public void updatePIDControl() {
        double error = targetPosition - currentPosition;
        integral += error * runtime.seconds();
        double derivative = (error - previousError) / runtime.seconds();

        double kP = 0.01;
        double kI = 0;
        double kD = 0.0;
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        floorSlide.setPower(power);
        previousError = error;
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
    }

    public double getCurrentPosition() {
        return floorSlide.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setFloorSlidePower(double power) {
        floorSlide.setPower(power);
    }

    public double getSlideAmps() {
        return floorSlide.getCurrent(CurrentUnit.AMPS);
    }

    public void update() {
        currentPosition = floorSlide.getCurrentPosition();
        updatePIDControl();
    }

    public void autoSlideMover(double autoTargetPosition){
        setTargetPosition(autoTargetPosition);
        while (Math.abs(autoTargetPosition - getCurrentPosition()) > 5){
            update();
        }
        floorSlide.setPower(0);
    }

    public double getSlideError() {
        return Math.abs(getTargetPosition()) - Math.abs(getCurrentPosition());
    }
}