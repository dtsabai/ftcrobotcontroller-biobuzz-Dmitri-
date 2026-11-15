package org.firstinspires.ftc.teamcode;
public class Sweep {
    public static void main(String[] a) {
        System.out.println("accel/s   reached  t(s)  overshoot  finalErr   [target (24,24,0), stiction 60]");
        for (double ac : new double[]{0, 3, 6, 10, 15, 20, 30, 50}) {
            Sim.Result r = Sim.run(24, 24, 0, 0.08, ac, 60, false, null);
            System.out.printf("%6.0f   %7s %5.2f %9.2f %9.2f%n", ac, r.reached, r.t, Math.max(0, r.maxOver), r.finalErr);
        }
    }
}
