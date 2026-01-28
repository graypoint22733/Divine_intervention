package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.Pose2d;

public class PedroPath {
    private final List<Pose2d> waypoints;
    private int index = 0;

    public PedroPath(List<Pose2d> waypoints) {
        if (waypoints == null || waypoints.isEmpty()) {
            throw new IllegalArgumentException("PedroPath requires at least one waypoint.");
        }
        this.waypoints = new ArrayList<>(waypoints);
    }

    public Pose2d getTarget() {
        return waypoints.get(index);
    }

    public boolean advanceIfReached(boolean reached) {
        if (reached && index < waypoints.size()) {
            index++;
        }
        return isFinished();
    }

    public boolean isFinished() {
        return index >= waypoints.size();
    }

    public void reset() {
        index = 0;
    }
}
