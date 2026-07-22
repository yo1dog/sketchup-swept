# frozen_string_literal: true
#
# simulation.rb — Holds simulation state, records the swept path, draws the
# live preview, and commits results to model geometry.

require 'json'

module Swept
  class Simulation
    DEG = Math::PI / 180.0
    SUBSTEP_M = 0.1 # integrate motion in small increments for accuracy

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

    attr_accessor :step_m, :ghost_spacing_m,
                  :show_body_traces, :show_wheel_tracks, :show_ghosts
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
        units: @vehicle.units.size
      }
    end

    def status_json
      status.to_json
    end

    # ---- Drawing (live preview) -----------------------------------------

    def draw(view)
      return unless @placed

      draw_traces(view)
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

      if @show_body_traces
        set_color(view, C_TRACE)
        view.line_width = 2
        each_body_trace { |poly| view.draw(GL_LINE_STRIP, poly.map { |c| Util.m_to_pt(c, Z_TRACE) }) }
      end

      return unless @show_wheel_tracks

      set_color(view, C_TRACK)
      view.line_width = 1
      each_mark_trace { |poly| view.draw(GL_LINE_STRIP, poly.map { |c| Util.m_to_pt(c, Z_TRACK) }) }
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

    # Yield a polyline for each traced body corner (per unit, per corner).
    def each_body_trace
      n_units = @frames.first[:fp].size
      n_units.times do |u|
        4.times do |c|
          poly = @frames.map { |f| f[:fp][u][:body][c] }
          yield poly
        end
      end
    end

    # Yield a polyline for each traced wheel-contact mark.
    def each_mark_trace
      n_units = @frames.first[:fp].size
      n_units.times do |u|
        n_marks = @frames.first[:fp][u][:marks].size
        n_marks.times do |m|
          poly = @frames.map { |f| f[:fp][u][:marks][m] }
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
      each_body_trace do |poly|
        add_polyline(ents, poly, Z_TRACE)
      end
    end

    def commit_tracks(ents)
      each_mark_trace do |poly|
        add_polyline(ents, poly, Z_TRACK)
      end
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
