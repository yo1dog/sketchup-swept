# frozen_string_literal: true
#
# simulation.rb — Holds simulation state, records the swept path, draws the
# live preview, and commits results to model geometry.

require 'json'

module Swept
  class Simulation
    DEG = Math::PI / 180.0
    SUBSTEP_M = 0.1 # integrate motion in small increments for accuracy
    ARC_SEG_DEG = 2.0 # committed arcs: roughly one segment per this many degrees

    # Draw layering (metres above ground) to avoid z-fighting.
    Z_TRACK = 0.010
    Z_TRACE = 0.020
    Z_GHOST = 0.005
    Z_BODY  = 0.030

    # Colours.
    C_BODY   = [30, 110, 220]
    C_WHEEL  = [40, 40, 40]
    C_TRACE  = [255, 140, 0]
    C_TRACK  = [0, 160, 70]
    C_GHOST  = [130, 170, 255]
    C_ARROW  = [220, 40, 40]
    C_HITCH  = [150, 60, 200]
    # Projection preview colours (forward = teal, reverse = magenta).
    C_PROJ_F  = [0, 200, 180]
    C_PROJ_FT = [0, 140, 120]
    C_PROJ_R  = [230, 90, 200]
    C_PROJ_RT = [170, 60, 150]

    attr_accessor :step_m, :ghost_spacing_m,
                  :show_body_traces, :show_wheel_tracks, :show_ghosts,
                  :show_projection_fwd, :show_projection_rev,
                  :project_mode, :project_distance_m, :project_steps
    attr_reader :frames, :dist_m, :steer_deg, :placed, :vehicle, :preset_key

    def initialize
      @preset = Presets::DEFAULT
      @preset_key = @preset[:key]
      @vehicle = Vehicle.new(@preset)
      @steer_deg = 0.0
      @step_m = 0.5
      @ghost_spacing_m = 2.0
      @show_body_traces = true
      @show_wheel_tracks = true
      @show_ghosts = true
      @show_projection_fwd = true
      @show_projection_rev = false
      @project_mode = :distance # :distance or :steps
      @project_distance_m = 8.0
      @project_steps = 10
      @frames = []
      @dist_m = 0.0
      @placed = false
      @origin = [0.0, 0.0]
      @heading0 = 0.0
    end

    # ---- Configuration ---------------------------------------------------

    def load_preset(key)
      @preset = Presets.find(key)
      @preset_key = @preset[:key]
      @vehicle = Vehicle.new(@preset)
      clamp_steer!
      replace_if_placed
    end

    def max_steer_deg
      @vehicle.max_steer / DEG
    end

    def steer_deg=(value)
      @steer_deg = value.to_f
      clamp_steer!
    end

    def steer_by(delta_deg)
      self.steer_deg = @steer_deg + delta_deg
    end

    # ---- Placement / motion ---------------------------------------------

    def place(pos_m, heading)
      @origin = pos_m.dup
      @heading0 = heading
      @placed = true
      @vehicle.place(pos_m, heading)
      @dist_m = 0.0
      @frames = [{ fp: @vehicle.footprint, dist: 0.0 }]
    end

    # Re-run from the recorded start point, clearing the swept history.
    def reset
      return unless @placed

      place(@origin, @heading0)
    end

    def clear
      @placed = false
      @frames = []
      @dist_m = 0.0
    end

    # Advance forward (ds > 0) or backward (ds < 0) by ds metres, recording a
    # footprint at every sub-step so the traces stay smooth on curves.
    def advance(ds)
      return unless @placed

      steer = @steer_deg * DEG
      n = [(ds.abs / SUBSTEP_M).ceil, 1].max
      inc = ds / n
      n.times do
        @vehicle.step(inc, steer)
        @dist_m += inc.abs
        @frames << { fp: @vehicle.footprint, dist: @dist_m }
      end
    end

    # ---- Projection (live preview of where the current steer leads) ------

    # How far ahead the projection reaches, in metres.
    def projection_distance
      @project_mode == :steps ? (@project_steps * @step_m) : @project_distance_m
    end

    # Simulate from the CURRENT pose at the CURRENT steering angle for the
    # projection distance, without recording into @frames or moving the real
    # vehicle. direction = +1 (forward) or -1 (reverse). Returns frames.
    def project(direction)
      return [] unless @placed

      dist = projection_distance
      return [] if dist <= 0

      steer = @steer_deg * DEG
      state = @vehicle.capture_state
      frames = [{ fp: @vehicle.footprint, dist: 0.0 }]
      n = [(dist / SUBSTEP_M).ceil, 1].max
      inc = direction * dist / n
      acc = 0.0
      n.times do
        @vehicle.step(inc, steer)
        acc += inc.abs
        frames << { fp: @vehicle.footprint, dist: acc }
      end
      @vehicle.restore_state(state)
      frames
    end

    # ---- Status readout --------------------------------------------------

    def status
      {
        placed: @placed,
        preset_key: @preset_key,
        frames: @frames.size,
        dist_m: round2(@dist_m),
        steer_deg: round2(@steer_deg),
        max_steer_deg: round2(max_steer_deg),
        radius_m: turn_radius_str,
        swept_width_m: round2(swept_width),
        step_m: @step_m,
        ghost_spacing_m: @ghost_spacing_m,
        units: @vehicle.units.size,
        project_fwd: @show_projection_fwd,
        project_rev: @show_projection_rev,
        project_mode: @project_mode.to_s,
        project_value: (@project_mode == :steps ? @project_steps : @project_distance_m),
        projection_len_m: round2(projection_distance)
      }
    end

    def status_json
      status.to_json
    end

    # ---- Drawing (live preview) -----------------------------------------

    def draw(view)
      return unless @placed

      draw_traces(view)
      draw_projection(view)
      draw_ghosts(view)
      draw_vehicle(view, @vehicle.footprint, Z_BODY, true)
    end

    # Draw a preview vehicle at an arbitrary pose without touching state.
    def draw_preview(view, pos_m, heading)
      ghost = Vehicle.new(@preset)
      ghost.place(pos_m, heading)
      draw_vehicle(view, ghost.footprint, Z_BODY, true)
    end

    # ---- Commit to model geometry ---------------------------------------

    def commit(model)
      return unless @placed && @frames.size > 1

      model.start_operation('Vehicle Swept Path', true)
      group = model.active_entities.add_group
      group.name = "Swept Path (#{@preset[:name]})"
      ents = group.entities

      commit_traces(ents) if @show_body_traces
      commit_tracks(ents) if @show_wheel_tracks
      commit_footprint_faces(ents)
      model.commit_operation
      group
    end

    private

    def clamp_steer!
      lim = max_steer_deg
      @steer_deg = lim if @steer_deg > lim
      @steer_deg = -lim if @steer_deg < -lim
    end

    def replace_if_placed
      place(@origin, @heading0) if @placed
    end

    def turn_radius_str
      steer = @steer_deg * DEG
      return '∞ (straight)' if steer.abs < 1e-6

      format('%.2f', (@vehicle.wheelbase / Math.tan(steer)).abs)
    end

    # Range of all body-corner points projected onto the axis perpendicular to
    # the starting heading — a practical "how wide did it sweep" measure.
    def swept_width
      return 0.0 if @frames.size < 2

      px = -Math.sin(@heading0)
      py = Math.cos(@heading0)
      min = nil
      max = nil
      @frames.each do |f|
        f[:fp].each do |unit|
          unit[:body].each do |c|
            d = (c[0] * px) + (c[1] * py)
            min = d if min.nil? || d < min
            max = d if max.nil? || d > max
          end
        end
      end
      (max - min)
    end

    def round2(v)
      (v * 100).round / 100.0
    end

    # --- live drawing helpers ---

    def set_color(view, rgb)
      view.drawing_color = Sketchup::Color.new(*rgb)
    end

    def draw_vehicle(view, fp, z, show_wheels)
      fp.each do |unit|
        pts = unit[:body].map { |c| Util.m_to_pt(c, z) }
        set_color(view, C_BODY)
        view.line_width = 3
        view.draw(GL_LINE_LOOP, pts)

        next unless show_wheels

        set_color(view, C_WHEEL)
        view.line_width = 4
        unit[:wheels].each do |seg|
          view.draw(GL_LINES, [Util.m_to_pt(seg[0], z), Util.m_to_pt(seg[1], z)])
        end
      end
      draw_heading_arrow(view, fp.first, z)
      draw_hitches(view, fp, z)
    end

    def draw_heading_arrow(view, unit, z)
      # From rear-axle-ish centre toward the front, an arrow along heading.
      b = unit[:body]
      rear_mid = mid(b[0], b[1])
      front_mid = mid(b[2], b[3])
      set_color(view, C_ARROW)
      view.line_width = 2
      view.draw(GL_LINES, [Util.m_to_pt(rear_mid, z), Util.m_to_pt(front_mid, z)])
    end

    def draw_hitches(view, fp, z)
      return if fp.size < 2

      set_color(view, C_HITCH)
      fp.each_cons(2) do |a, b|
        # Connect the front unit's rear-mid to the towed unit's front-mid.
        pa = mid(a[:body][0], a[:body][1])
        pb = mid(b[:body][2], b[:body][3])
        view.line_width = 2
        view.draw(GL_LINES, [Util.m_to_pt(pa, z), Util.m_to_pt(pb, z)])
      end
    end

    def draw_traces(view)
      return if @frames.size < 2

      draw_outline(view, @frames, C_TRACE, Z_TRACE) if @show_body_traces

      return unless @show_wheel_tracks

      set_color(view, C_TRACK)
      view.line_width = 1
      each_mark_trace { |poly| view.draw(GL_LINE_STRIP, poly.map { |c| Util.m_to_pt(c, Z_TRACK) }) }
    end

    # Draw the forward/reverse projection previews as solid lines.
    def draw_projection(view)
      #view.line_stipple = '-'
      draw_proj_dir(view, project(1), C_PROJ_F, C_PROJ_FT) if @show_projection_fwd
      draw_proj_dir(view, project(-1), C_PROJ_R, C_PROJ_RT) if @show_projection_rev
      #view.line_stipple = ''
    end

    def draw_proj_dir(view, frames, body_c, track_c)
      return if frames.size < 2

      draw_outline(view, frames, body_c, Z_TRACE) if @show_body_traces

      if @show_wheel_tracks
        set_color(view, track_c)
        view.line_width = 1
        each_mark_trace(frames) do |poly|
          view.draw(GL_LINE_STRIP, poly.map { |c| Util.m_to_pt(c, Z_TRACK) })
        end
      end

      # Outline the projected end position.
      set_color(view, body_c)
      view.line_width = 1
      frames.last[:fp].each do |unit|
        view.draw(GL_LINE_LOOP, unit[:body].map { |c| Util.m_to_pt(c, Z_GHOST) })
      end
    end

    def draw_ghosts(view)
      return unless @show_ghosts

      set_color(view, C_GHOST)
      view.line_width = 1
      ghost_frames.each do |f|
        f[:fp].each do |unit|
          pts = unit[:body].map { |c| Util.m_to_pt(c, Z_GHOST) }
          view.draw(GL_LINE_LOOP, pts)
        end
      end
    end

    # Draw the swept-area envelope for the given frames: for each vehicle unit,
    # the outer and inner rails (as polylines) plus the start/end footprint caps.
    def draw_outline(view, frames, color, z)
      set_color(view, color)
      view.line_width = 2
      body_envelopes(frames).each do |env|
        view.draw(GL_LINE_STRIP, env[:left].map { |p| Util.m_to_pt(p, z) }) if env[:left].size > 1
        view.draw(GL_LINE_STRIP, env[:right].map { |p| Util.m_to_pt(p, z) }) if env[:right].size > 1
        env[:swingout].each do |arc|
          view.draw(GL_LINE_STRIP, arc_points_m(arc).map { |p| Util.m_to_pt(p, z) })
        end
        [env[:first], env[:last]].each do |cap|
          view.draw(GL_LINE_LOOP, cap.map { |p| Util.m_to_pt(p, z) })
        end
      end
    end

    # Build the swept-area envelope for every unit. The true swept region of a
    # rigid body is bounded by two rails plus the terminal footprints:
    #   * the OUTER rail is traced by the body corner farthest from the
    #     instantaneous centre of rotation (ICR),
    #   * the INNER rail by the nearest point of the body to the ICR,
    #   * the start and end footprints cap the region.
    # This is analytic (no sampling/union): the ICR at each step comes directly
    # from the change in pose. Returns one hash per unit:
    #   { left:, right:, centres:, first:, last: }  (rails as point arrays,
    #   centres[i] = the ICR that generated rail point i, or nil when straight).
    def body_envelopes(frames)
      return [] if frames.size < 2

      frames.first[:fp].each_index.map { |u| unit_envelope(frames, u) }
    end

    def unit_envelope(frames, unit_idx)
      bodies = frames.map { |f| f[:fp][unit_idx][:body] }
      n = bodies.size
      left = []
      right = []
      centres = []
      n.times do |i|
        # Use the forward step (last frame reuses the final step) so the turn
        # direction — which side is inner vs outer — is signed correctly.
        a = [i, n - 2].min
        centre, dth = Util.pose_icr(bodies[a], bodies[a + 1])
        body = bodies[i]
        if centre.nil?
          left << body[3]  # front-left
          right << body[2] # front-right
        else
          far = Util.far_corner(centre, body)
          near = Util.near_point(centre, body)
          # Left turn (dth > 0): ICR is on the left, so the near point is the
          # left/inner rail and the far corner the right/outer rail; mirror for
          # a right turn. This keeps each rail on a consistent physical side.
          if dth.positive?
            left << near
            right << far
          else
            left << far
            right << near
          end
        end
        centres << centre
      end
      { left: left, right: right, centres: centres, first: bodies.first,
        last: bodies.last, swingout: swingout_arcs(bodies, centres) }
    end

    # Rear-overhang swingout (tail swing): while a vehicle turns, its rear-outer
    # corner rotates about the fixed instantaneous centre of rotation (ICR),
    # tracing a circular arc that bulges outside the rest of the swept area on
    # turn entry. The outer rail follows the farthest corner and cannot also
    # capture this, so add it as a separate arc. It is fully analytic — no
    # sampling. Returns an array of arc specs { centre:, radius:, a0:, sweep: },
    # one per turn leg (a maximal run of frames sharing an ICR).
    def swingout_arcs(bodies, centres)
      n = bodies.size
      arcs = []
      n.times do |i|
        centre = centres[i]
        next unless centre
        next unless i.zero? || !same_centre?(centres[i - 1], centre)

        # Cap the arc at the leg's actual rotation (short turns end before the
        # corner would rejoin the swept area).
        j = i
        j += 1 while j + 1 < n && centres[j + 1] && same_centre?(centres[j + 1], centre)
        max_sweep = Util.norm(Util.rect_heading(bodies[j]) - Util.rect_heading(bodies[i])).abs
        arc = swingout_arc(bodies[i], centre, max_sweep)
        arcs << arc if arc
      end
      arcs
    end

    # Swingout arc for one turn-leg start footprint, or nil if the rear corner
    # does not protrude. The arc starts at the rear-outer corner and ends where
    # the same circle (about the ICR) re-crosses the footprint's outer edge:
    #   P* = RO + s*d,  d = unit(RO->FO),  s = -2 (RO - ICR)·d.
    def swingout_arc(body, centre, max_sweep)
      ro = [body[0], body[1]].max_by { |p| dist2(p, centre) } # outer-side rear corner
      fo = [body[2], body[3]].max_by { |p| dist2(p, centre) } # outer-side front corner
      dx = fo[0] - ro[0]
      dy = fo[1] - ro[1]
      len = Math.hypot(dx, dy)
      return nil if len < 1e-9

      ux = dx / len
      uy = dy / len
      s = -2.0 * (((ro[0] - centre[0]) * ux) + ((ro[1] - centre[1]) * uy))
      return nil unless s > 1e-6 && s <= len + 1e-9 # rear corner does not swing out

      pstar = [ro[0] + (ux * s), ro[1] + (uy * s)]
      a0 = Math.atan2(ro[1] - centre[1], ro[0] - centre[0])
      sweep = Util.norm(Math.atan2(pstar[1] - centre[1], pstar[0] - centre[0]) - a0)
      return nil if sweep.abs < 1e-6

      sweep = (sweep.positive? ? 1 : -1) * [sweep.abs, max_sweep].min
      { centre: centre, radius: Math.sqrt(dist2(ro, centre)), a0: a0, sweep: sweep }
    end

    def dist2(a, b)
      ((a[0] - b[0])**2) + ((a[1] - b[1])**2)
    end

    # Tessellate an arc spec into a polyline of metre points for preview.
    def arc_points_m(arc)
      steps = [(arc[:sweep].abs / (ARC_SEG_DEG * DEG)).ceil, 1].max
      (0..steps).map do |k|
        ang = arc[:a0] + (arc[:sweep] * k / steps)
        [arc[:centre][0] + (arc[:radius] * Math.cos(ang)),
         arc[:centre][1] + (arc[:radius] * Math.sin(ang))]
      end
    end

    # Yield a polyline for each traced wheel-contact mark.
    def each_mark_trace(frames = @frames)
      return if frames.size < 2

      n_units = frames.first[:fp].size
      n_units.times do |u|
        n_marks = frames.first[:fp][u][:marks].size
        n_marks.times do |m|
          poly = frames.map { |f| f[:fp][u][:marks][m] }
          yield poly
        end
      end
    end

    # Footprints spaced roughly every ghost_spacing_m of travel (plus the last).
    def ghost_frames
      out = []
      next_at = 0.0
      @frames.each do |f|
        if f[:dist] >= next_at
          out << f
          next_at = f[:dist] + @ghost_spacing_m
        end
      end
      out << @frames.last unless out.include?(@frames.last)
      out
    end

    def mid(a, b)
      [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
    end

    # --- commit helpers ---

    def commit_traces(ents)
      body_envelopes(@frames).each do |env|
        commit_rail(ents, env[:left], env[:centres])
        commit_rail(ents, env[:right], env[:centres])
        env[:swingout].each { |arc| commit_swingout_arc(ents, arc) }
        add_polyline(ents, env[:first] + [env[:first].first], Z_TRACE)
        add_polyline(ents, env[:last] + [env[:last].first], Z_TRACE)
      end
    end

    # Commit one polyline whose points share a per-frame ICR (a rail or a wheel
    # track). Consecutive points sharing an ICR are a constant-radius arc, so
    # emit a true arc entity; straight/mixed stretches become edges.
    def commit_rail(ents, pts, centres, z = Z_TRACE)
      n = pts.size
      return if n < 2

      i = 0
      while i < n - 1
        centre = centres[i]
        run_end = arc_run_end(centre, centres, i, n)
        if run_end - i >= 2 && commit_arc(ents, pts[i..run_end], centre, z)
          i = run_end
        else
          add_polyline(ents, [pts[i], pts[i + 1]], z)
          i += 1
        end
      end
    end

    # Extent of the maximal run starting at i whose points share centre.
    def arc_run_end(centre, centres, i, n)
      return i if centre.nil?

      j = i
      j += 1 while j + 1 < n && centres[j + 1] && same_centre?(centres[j + 1], centre)
      j
    end

    def same_centre?(a, b)
      ((a[0] - b[0]).abs < 1e-4) && ((a[1] - b[1]).abs < 1e-4)
    end

    # Emit a true circular arc through run points about centre. Returns false
    # (caller falls back to edges) if SketchUp rejects the geometry.
    def commit_arc(ents, run, centre, z = Z_TRACE)
      radius = run.sum { |p| Math.hypot(p[0] - centre[0], p[1] - centre[1]) } / run.size
      sweep = 0.0
      run.each_cons(2) do |a, b|
        va = [a[0] - centre[0], a[1] - centre[1]]
        vb = [b[0] - centre[0], b[1] - centre[1]]
        sweep += Math.atan2((va[0] * vb[1]) - (va[1] * vb[0]), (va[0] * vb[0]) + (va[1] * vb[1]))
      end
      return false if sweep.abs < 1e-6 || radius < 1e-6

      c3 = Util.m_to_pt(centre, z)
      xaxis = Geom::Vector3d.new(run.first[0] - centre[0], run.first[1] - centre[1], 0)
      normal = Geom::Vector3d.new(0, 0, sweep.positive? ? 1 : -1)
      ents.add_arc(c3, xaxis, normal, radius * Util::IN_PER_M, 0.0, sweep.abs, arc_segments(sweep))
      true
    rescue StandardError
      false
    end

    # Segment count for a committed arc of the given sweep (radians) — finer
    # than SketchUp's default of 12 so long arcs stay smooth.
    def arc_segments(sweep)
      [(sweep.abs / (ARC_SEG_DEG * DEG)).ceil, 8].max
    end

    # Commit a swingout arc spec as a real arc entity (polyline on failure).
    def commit_swingout_arc(ents, arc)
      c3 = Util.m_to_pt(arc[:centre], Z_TRACE)
      xaxis = Geom::Vector3d.new(Math.cos(arc[:a0]), Math.sin(arc[:a0]), 0)
      normal = Geom::Vector3d.new(0, 0, arc[:sweep].positive? ? 1 : -1)
      ents.add_arc(c3, xaxis, normal, arc[:radius] * Util::IN_PER_M, 0.0, arc[:sweep].abs,
                   arc_segments(arc[:sweep]))
    rescue StandardError
      add_polyline(ents, arc_points_m(arc), Z_TRACE)
    end

    # Wheel-contact marks are fixed points on each rigid unit, so they rotate
    # about that unit's ICR just like the body — commit them with the same
    # arc-fitting so tracks are real arcs on constant-steer legs.
    def commit_tracks(ents)
      @frames.first[:fp].each_index do |u|
        centres = unit_centres(@frames.map { |f| f[:fp][u][:body] })
        n_marks = @frames.first[:fp][u][:marks].size
        n_marks.times do |m|
          poly = @frames.map { |f| f[:fp][u][:marks][m] }
          commit_rail(ents, poly, centres, Z_TRACK)
        end
      end
    end

    # Per-frame ICR of a unit (nil where the motion is straight), aligned with
    # the frame list — the same centres used for the body rails.
    def unit_centres(bodies)
      n = bodies.size
      Array.new(n) { |i| Util.pose_icr(bodies[[i, n - 2].min], bodies[[i, n - 2].min + 1]).first }
    end

    def commit_footprint_faces(ents)
      mat = ghost_material(ents.model)
      ghost_frames.each do |f|
        f[:fp].each do |unit|
          pts = unit[:body].map { |c| Util.m_to_pt(c, Z_GHOST) }
          begin
            face = ents.add_face(pts)
            face.material = mat if face && mat
            face.back_material = mat if face && mat
          rescue StandardError
            # Degenerate footprint — skip.
          end
        end
      end
    end

    def add_polyline(ents, poly, z)
      pts = poly.map { |c| Util.m_to_pt(c, z) }
      ents.add_edges(pts) if pts.size > 1
    rescue StandardError
      nil
    end

    def ghost_material(model)
      name = 'Swept Path Footprint'
      mat = model.materials[name] || model.materials.add(name)
      mat.color = Sketchup::Color.new(*C_GHOST)
      mat.alpha = 0.25
      mat
    rescue StandardError
      nil
    end
  end
end
