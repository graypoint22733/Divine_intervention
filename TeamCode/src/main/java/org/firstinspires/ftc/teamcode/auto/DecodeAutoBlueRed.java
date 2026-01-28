package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Decode Pedro Auto - Blue/Red", group = "Auton")
public class DecodeAutoBlueRed extends DecodePedroAutoBase {
    @Override
    protected DecodePedroAutoPlan.Alliance getAlliance() {
        return DecodePedroAutoPlan.Alliance.BLUE;
    }

    @Override
    protected DecodePedroAutoPlan.StartSide getStartSide() {
        return DecodePedroAutoPlan.StartSide.RED;
    }
}
