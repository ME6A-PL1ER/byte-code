package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class FloorGrippySubsystem extends SubsystemBase {
    private final Servo floorServo;

    public FloorGrippySubsystem(Servo floorServo) {
        this.floorServo = floorServo;
    }


    public void setServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        floorServo.setPosition(mappedPosition);
    }

}
