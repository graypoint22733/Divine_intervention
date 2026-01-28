package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Decode Pedro Auto - Red/Red", group = "Auton")
public class DecodeAutoRedRed extends DecodePedroAutoBase {
    @Override
    protected DecodePedroAutoPlan.Alliance getAlliance() {
        return DecodePedroAutoPlan.Alliance.RED;
    }

    @Override
    protected DecodePedroAutoPlan.StartSide getStartSide() {
        return DecodePedroAutoPlan.StartSide.RED;
    }
}
