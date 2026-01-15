package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDF;

public class Turret {
    public static final double GEAR_RATIO = 80/30*37/112, SERVO_TO_ANGLE = 300;

    private Servo LServo;
    private Servo RServo;

    public double target;
    private boolean tracking = false;

    public Turret (HardwareMap map) {
        LServo = map.get(Servo.class, "TurretServoL");
        RServo = map.get(Servo.class, "TurretServoR");
    }

    public void update(){
        if (tracking) {

        }
    }

    private void setPos(double pos){
        LServo.setPosition(pos);
        RServo.setPosition(pos);
    }

    public double getPosition(){
        return LServo.getPosition();
    }

    public double getAngle(){
        return LServo.getPosition() * SERVO_TO_ANGLE;
    }

    public void setAngle(double angle){
        setPos(angle / SERVO_TO_ANGLE);
    }

    public void setPosition(double pos){
        setPos(pos);
    }

    public void startTracking(){tracking = true;}
    public void stopTracking(){tracking = false;}
}
