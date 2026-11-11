import clunkyard.*;

/**
 * Helix: range-gated spiral pursuit, on the Java binding (protocol 2).
 *
 * It closes on the nearest robot along a spiral whose hand flips every 1.7
 * seconds, backs off inside 180 and fires only between 90 and 490, with
 * shots whose power suits the range, led on the target's tracked motion.
 * With nothing in view it spirals in toward the middle, sweeping a narrow
 * scan, and turns toward a cue it hears. It steps out across the storm's
 * path and keeps to the safe zone in sudden death.
 */
public final class Helix extends Robot {
    private static final Vec3 ORIGIN = new Vec3(0, 0, 0);

    /** Out across the storm's path, on the side away from its centre. */
    private double stormEscape(Storm storm) {
        double away = angles().bearingTo(storm.center(), me().position());
        Vec3 v = storm.velocity();
        if (Math.hypot(v.x(), v.y()) < 1) return away;
        double drift = angles().bearingTo(ORIGIN, v);
        double side = drift + (angles().angleDiff(away, drift) < 0 ? -90 : 90);
        return side + angles().angleDiff(away, side) / 3;
    }

    @Override protected void onTurn() {
        int tick = clock().tick();
        boolean three = match().is3d(), forces = match().control() == Control.FORCES;
        double heading = me().heading();
        Vec3 pos = me().position();
        double spiral = (tick / 34) % 2 == 0 ? 1 : -1;
        var nearest = sensors().nearest();
        Contact target = nearest.orElse(null);
        double range = target == null ? Double.POSITIVE_INFINITY : target.range();

        double turn, throttle = range < 180 ? -0.35 : 0.8, pitchTurn = three ? 0.17 * Math.sin(tick / 20.0) : 0;
        if (target != null) {
            boolean ready = range > 90 && range < 490 && gun().cooldownTicks() <= 2;
            // The spiral eases as the gun comes ready, so the shot leaves a steadier hull.
            turn = angles().clamp(angles().angleDiff(target.bearing(), heading) / 90 + spiral * (ready ? 0.08 : 0.26));
            double power = gun().powerForFlightTime(range, range > 300 ? 1.4 : 1.1);
            var aim = gun().lead(target, power);
            double aimHeading = aim.map(Solution::heading).orElse(target.bearing());
            gun().aim(aimHeading, three ? aim.map(Solution::pitch).orElse(target.elevation()) : 0);
            // The turret slews, so fire only once it points at the target.
            if (ready) throttle = Math.min(throttle, 0.35);
            if (range > 90 && range < 490 && Math.abs(angles().angleDiff(aimHeading, gun().aimNow().heading())) < 4 && gun().canFire(power, 6))
                gun().fire(power);
            scanner().set(target.bearing(), 16, target.elevation());
            status().set("HELIX", range);
        } else {
            var cue = tracker().cue();
            Vec3 size = arena().size();
            Vec3 toward = cue.map(Cue::position).orElse(new Vec3(size.x() / 2, size.y() / 2, pos.z()));
            turn = angles().clamp(angles().angleDiff(angles().bearingTo(pos, toward), heading) / 90 + spiral * 0.3);
            if (cue.isPresent()) scanner().set(angles().bearingTo(pos, toward) + (tick % 5 - 2) * Math.min(cue.get().bearingError(), 12), 30, 0);
            else scanner().set((tick * 27.0) % 360, 30, three ? 24 * Math.sin(tick / 22.0) : 0);
            status().set("SPIRAL", tick);
        }
        // The storm and sudden death take the wheel; the gun keeps working.
        // The way out, if the storm or the zone has the wheel, or NaN.
        double way = Double.NaN;
        var storm = nav().storm();
        if (storm.isPresent() && storm.get().margin(pos).orElse(1e9) > -70) {
            way = stormEscape(storm.get());
            turn = angles().clamp(angles().angleDiff(way, heading) / 45);
            throttle = 1;
        }
        var zone = nav().zone();
        double zoneMargin = zone.map(z -> z.margin(pos).orElse(-1e9)).orElse(1e9);
        // Near the zone's edge it heads in, unless it is fleeing the storm and still inside.
        if (zoneMargin < 60 && (Double.isNaN(way) || zoneMargin < 0)) {
            way = angles().bearingTo(pos, zone.get().center());
            turn = angles().clamp(angles().angleDiff(way, heading) / 45);
            if (zone.get().shape() == Shape.SPHERE) pitchTurn = angles().clamp((angles().elevationTo(pos, zone.get().center()) - me().pitch()) / 20);
            throttle = 0.8;
        }
        // A low battery drives gently, so the shots can still be paid for,
        // unless the storm or the zone has the wheel.
        if (Double.isNaN(way) && battery().charge() < battery().capacity() * 0.4) throttle *= 0.5;
        if (!forces) {
            drive().set(throttle, turn, pitchTurn);
            return;
        }
        Vec3 velocity = me().velocity();
        double lift = 0;
        if (three) {
            double g = arena().gravity().z();
            lift = g != 0
                ? -g / 110 + (arena().groundHeight(pos.x(), pos.y()) + 110 + spiral * 20 - pos.z()) / 122 - velocity.z() / 66
                : -velocity.z() / 48 + spiral * 0.08;
        }
        double speed = Math.sqrt(velocity.x() * velocity.x() + velocity.y() * velocity.y() + velocity.z() * velocity.z());
        if (!Double.isNaN(way)) {
            // Push straight along the way out, whatever the hull's heading.
            Vec3 out = angles().direction(way, 0);
            Attitude axes = me().attitude();
            drive().forces(angles().clamp(out.x() * axes.forward().x() + out.y() * axes.forward().y()),
                angles().clamp(out.x() * axes.right().x() + out.y() * axes.right().y()), angles().clamp(lift),
                angles().clamp(turn - me().angularVelocity().yaw() / 60), 0, 0, 0);
            return;
        }
        // Under forces it holds about 55, since spread grows with its own speed.
        drive().forces(speed < 55 ? throttle : 0, spiral * 0.31, angles().clamp(lift),
            angles().clamp(turn - me().angularVelocity().yaw() / 140), pitchTurn, 0, speed > 62 ? 0.72 : 0);
    }

    public static void main(String[] args) {
        Clunkyard.run(new Helix());
    }
}
