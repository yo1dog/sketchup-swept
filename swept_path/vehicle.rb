# frozen_string_literal: true
#
# vehicle.rb — Kinematic vehicle model (bicycle model + towed-unit chain).
#
# Coordinate conventions (all in metres, all 2D on the z=0 plane):
#   * A pose is a reference point [x, y] plus a heading angle (radians).
#   * Local frame of a unit: +x points forward (along heading), +y points left.
#   * Lead unit reference point = centre of its REAR axle.
#   * Towed unit reference point = centre of its (single effective) axle.
#
# The lead unit uses the standard kinematic bicycle model. Each towed unit is
# pulled by a hitch point rigidly attached to the unit in front, and follows a
# non-holonomic (no side-slip) constraint that produces realistic off-tracking
# and, when reversing, jackknifing.

module Swept
  module Util
    IN_PER_M = 1.0 / 0.0254 # SketchUp's internal unit is the inch

    module_function

    # Normalise an angle into (-pi, pi].
    def norm(a)
      a %= (2 * Math::PI)
      a -= 2 * Math::PI if a > Math::PI
      a
    end

    # Transform a local point (lx, ly) into world coords given a reference
    # point rp = [x, y] and heading h.
    def l2w(rp, h, lx, ly)
      c = Math.cos(h)
      s = Math.sin(h)
      [rp[0] + (lx * c) - (ly * s), rp[1] + (lx * s) + (ly * c)]
    end

    # Convert a metres [x, y] (optionally with metre z) into a Geom::Point3d in
    # SketchUp internal units.
    def m_to_pt(xy, z_m = 0.0)
      Geom::Point3d.new(xy[0] * IN_PER_M, xy[1] * IN_PER_M, z_m * IN_PER_M)
    end

    # Convert a Geom::Point3d back into a metres [x, y] pair (drops z).
    def pt_to_m(pt)
      [pt.x.to_f / IN_PER_M, pt.y.to_f / IN_PER_M]
    end

    # --- 2D geometry helpers (used to build the swept-area envelope) ------
    #
    # A "body" here is a footprint rectangle: 4 world corners in the order
    # [rear-left, rear-right, front-right, front-left] (see Unit#footprint).

    def mid(a, b)
      [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
    end

    # Instantaneous centre of rotation (ICR) that carries body b0 onto b1, and
    # the heading change dtheta. Returns [centre, dtheta], or [nil, dtheta] when
    # the motion is a pure translation (straight). Exact for constant-curvature
    # motion: a rigid body's poses at two instants determine the fixed point it
    # rotates about.
    def pose_icr(b0, b1)
      r0 = mid(b0[0], b0[1]) # rear-axle midpoint before
      r1 = mid(b1[0], b1[1]) # rear-axle midpoint after
      dth = norm(rect_heading(b1) - rect_heading(b0))
      return [nil, dth] if dth.abs < 1e-6

      # (r1 - c) = Rot(dth)(r0 - c)  =>  c = r0 - (Rot(dth) - I)^-1 (r1 - r0)
      cc = Math.cos(dth) - 1.0
      ss = Math.sin(dth)
      det = (cc * cc) + (ss * ss)
      dx = r1[0] - r0[0]
      dy = r1[1] - r0[1]
      ix = ((cc * dx) + (ss * dy)) / det
      iy = ((-ss * dx) + (cc * dy)) / det
      [[r0[0] - ix, r0[1] - iy], dth]
    end

    # Heading (radians) of a footprint rectangle, rear-midpoint -> front-midpoint.
    def rect_heading(body)
      rm = mid(body[0], body[1])
      fm = mid(body[2], body[3])
      Math.atan2(fm[1] - rm[1], fm[0] - rm[0])
    end

    # The body corner farthest from centre c (traces the outer envelope arc).
    def far_corner(c, body)
      body.max_by { |p| ((p[0] - c[0])**2) + ((p[1] - c[1])**2) }
    end

    # The point on the body rectangle's boundary nearest to centre c (traces the
    # inner envelope arc). May be a corner or the perpendicular foot on an edge.
    def near_point(c, body)
      best = nil
      4.times do |i|
        a = body[i]
        b = body[(i + 1) % 4]
        ax = b[0] - a[0]
        ay = b[1] - a[1]
        len2 = (ax * ax) + (ay * ay)
        t = len2 < 1e-12 ? 0.0 : (((c[0] - a[0]) * ax) + ((c[1] - a[1]) * ay)) / len2
        t = 0.0 if t < 0.0
        t = 1.0 if t > 1.0
        p = [a[0] + (ax * t), a[1] + (ay * t)]
        d2 = ((p[0] - c[0])**2) + ((p[1] - c[1])**2)
        best = [d2, p] if best.nil? || d2 < best[0]
      end
      best[1]
    end
  end

  # A single rigid unit of a vehicle (lead or towed).
  class Unit
    DEG = Math::PI / 180.0

    attr_reader :kind, :width, :max_steer, :heading

    def initialize(spec)
      @kind           = spec[:kind]
      @wheelbase      = spec[:wheelbase]
      @drawbar        = spec[:drawbar]
      @front_overhang = spec[:front_overhang]
      @rear_overhang  = spec[:rear_overhang]
      @width          = spec[:width]
      @track          = spec[:track] || (spec[:width] * 0.85)
      @hitch_offset   = spec[:hitch_offset]

      @max_steer =
        if spec[:min_turn_radius] && @wheelbase
          Math.atan(@wheelbase / spec[:min_turn_radius])
        else
          35 * DEG
        end

      @pos = [0.0, 0.0]
      @axle = [0.0, 0.0]
      @heading = 0.0
      @last_steer = 0.0
    end

    def lead?
      @kind == :lead
    end

    def reference_point
      lead? ? @pos : @axle
    end

    # World position of the hitch point where the NEXT unit attaches.
    def hitch_point
      off = @hitch_offset || 0.0
      Util.l2w(reference_point, @heading, off, 0.0)
    end

    # ---- Placement -------------------------------------------------------

    def init_lead(pos, heading)
      @pos = pos.dup
      @heading = Util.norm(heading)
      @last_steer = 0.0
    end

    def init_towed(hitch, heading)
      @heading = Util.norm(heading)
      @axle = [hitch[0] - (@drawbar * Math.cos(@heading)),
               hitch[1] - (@drawbar * Math.sin(@heading))]
    end

    # Snapshot / restore the mutable pose so a projection can be simulated
    # forward from the current state and then rolled back.
    def capture
      { pos: @pos.dup, axle: @axle.dup, heading: @heading, last_steer: @last_steer }
    end

    def restore(s)
      @pos = s[:pos].dup
      @axle = s[:axle].dup
      @heading = s[:heading]
      @last_steer = s[:last_steer]
    end

    # ---- Motion ----------------------------------------------------------

    # Advance the lead unit by ds metres (may be negative) at steer radians.
    def step_lead(ds, steer)
      steer = clamp(steer, -@max_steer, @max_steer)
      @last_steer = steer
      if steer.abs < 1e-9
        @pos[0] += ds * Math.cos(@heading)
        @pos[1] += ds * Math.sin(@heading)
      else
        r = @wheelbase / Math.tan(steer)
        nh = @heading + (ds / r)
        @pos[0] += r * (Math.sin(nh) - Math.sin(@heading))
        @pos[1] -= r * (Math.cos(nh) - Math.cos(@heading))
        @heading = Util.norm(nh)
      end
    end

    # Advance a towed unit given the hitch's motion (old -> new world points).
    def step_towed(old_hitch, new_hitch)
      dh = [new_hitch[0] - old_hitch[0], new_hitch[1] - old_hitch[1]]
      perp = [-Math.sin(@heading), Math.cos(@heading)]
      dtheta = ((dh[0] * perp[0]) + (dh[1] * perp[1])) / @drawbar
      @heading = Util.norm(@heading + dtheta)
      @axle = [new_hitch[0] - (@drawbar * Math.cos(@heading)),
               new_hitch[1] - (@drawbar * Math.sin(@heading))]
    end

    # ---- Geometry --------------------------------------------------------

    # Returns a hash describing the unit's current footprint (all in metres):
    #   :body   4 world corners [rear-left, rear-right, front-right, front-left]
    #   :wheels array of [p0, p1] short segments for drawing tyres
    #   :marks  world centres of the wheels whose tracks we trace
    def footprint
      rp = reference_point
      h = @heading
      xf = lead? ? (@wheelbase + @front_overhang) : @front_overhang
      xr = -@rear_overhang
      hw = @width / 2.0
      tw = @track / 2.0

      body = [
        Util.l2w(rp, h, xr, hw),   # rear-left
        Util.l2w(rp, h, xr, -hw),  # rear-right
        Util.l2w(rp, h, xf, -hw),  # front-right
        Util.l2w(rp, h, xf, hw)    # front-left
      ]

      wheels = []
      marks = []
      # Rear axle wheels (roll along heading).
      add_wheel(wheels, marks, rp, h, 0.0, tw, h)
      add_wheel(wheels, marks, rp, h, 0.0, -tw, h)
      if lead?
        # Front axle wheels (roll along heading + steer).
        fh = h + @last_steer
        add_wheel(wheels, marks, rp, h, @wheelbase, tw, fh)
        add_wheel(wheels, marks, rp, h, @wheelbase, -tw, fh)
      end

      { body: body, wheels: wheels, marks: marks }
    end

    private

    def add_wheel(wheels, marks, rp, h, lx, ly, roll_heading, len = 0.7)
      centre = Util.l2w(rp, h, lx, ly)
      dx = Math.cos(roll_heading) * (len / 2.0)
      dy = Math.sin(roll_heading) * (len / 2.0)
      wheels << [[centre[0] - dx, centre[1] - dy], [centre[0] + dx, centre[1] + dy]]
      marks << centre
    end

    def clamp(v, lo, hi)
      return lo if v < lo
      return hi if v > hi

      v
    end
  end

  # A vehicle is an ordered chain of units.
  class Vehicle
    attr_reader :units

    def initialize(preset)
      @units = preset[:units].map { |spec| Unit.new(spec) }
    end

    def lead
      @units.first
    end

    def max_steer
      lead.max_steer
    end

    def wheelbase
      lead.instance_variable_get(:@wheelbase)
    end

    def capture_state
      @units.map(&:capture)
    end

    def restore_state(state)
      @units.each_with_index { |u, i| u.restore(state[i]) }
    end

    def place(pos, heading)
      lead.init_lead(pos, heading)
      (1...@units.size).each do |i|
        prev = @units[i - 1]
        @units[i].init_towed(prev.hitch_point, prev.heading)
      end
    end

    # Advance the whole chain by ds metres at the given steer angle (radians).
    def step(ds, steer)
      old_hitches = @units.map(&:hitch_point) # capture BEFORE anything moves
      lead.step_lead(ds, steer)
      (1...@units.size).each do |i|
        @units[i].step_towed(old_hitches[i - 1], @units[i - 1].hitch_point)
      end
    end

    # Footprint of the whole vehicle: array of per-unit footprint hashes.
    def footprint
      @units.map(&:footprint)
    end
  end
end
