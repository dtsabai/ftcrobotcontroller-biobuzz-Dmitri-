package org.firstinspires.ftc.teamcode;
public class Dead {
    public static void main(String[] a) {
        double[][] wp = {{24,0,0},{24,24,0},{60,40,90},{3,0,0}};
        System.out.println("deadband  minHold  stiction  waypoint     final err(in)  speed in last 2s (in/s, 0=settled)");
        for (double depth : new double[]{0.9, 0.0}) for (double mh : new double[]{0, 0.08}) for (double st : new double[]{0, 150}) {
            Sim.deadDepth = depth;
            for (double[] w : wp) {
                Sim.Result r = Sim.run(w[0], w[1], w[2], mh, 6, st, false, null);
                System.out.printf("%7.1f  %7.2f  %8.0f  %-11s  %12.2f  %10.2f%n", depth, mh, st, String.format("(%.0f,%.0f,%.0f)", w[0], w[1], w[2]), r.finalErr, Sim.lastSpeedTail);
            }
        }
    }
}