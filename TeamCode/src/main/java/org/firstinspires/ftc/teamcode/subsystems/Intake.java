package org.firstinspires.ftc.teamcode.subsystems;
 
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
 
import org.firstinspires.ftc.teamcode.util.templates.Feature;
 
import java.util.ArrayList;
 
public class Intake extends Feature {
 
    private DcMotorEx motor;
    private double pow = 0;
 
    public Intake (HardwareMap map) {
        super(new ArrayList<>());
 
        motor = map.get(DcMotorEx.class, "intake");
    }
 
    @Override
    public void update() {
        motor.setPower(pow);
    }
 
    public void setVelocity(double target) {
        pow = target;
    }
 
    @Override
    public String toString() {
        return "Intake{" +
                ", pow=" + pow +
                '}';
    }
}