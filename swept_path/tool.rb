# frozen_string_literal: true
#
# tool.rb — The interactive SketchUp Tool.
#
# Placement flow:
#   1. Click once to set the vehicle's start point (rear axle of the lead unit).
#   2. Move the mouse and click again to set the initial heading.
#   3. Steer + drive with the arrow keys (or the control panel):
#        Up / Down    : drive forward / backward one step
#        Left / Right : steer left / right
#        Space        : centre the steering
#        Esc          : re-place the vehicle
#
# The tool draws the live preview; the shared Simulation (Swept::App.sim) owns
# the actual state so the HtmlDialog control panel and the keyboard stay in sync.

module Swept
  class SweptPathTool
    DEG = Math::PI / 180.0

    def initialize
      @sim = App.sim
      @stage = :place_origin
      @origin = nil
      @mouse_m = nil
    end

    # ---- Tool lifecycle --------------------------------------------------

    def activate
      @stage = @sim.placed ? :ready : :place_origin
      update_ui
      set_status
    end

    def deactivate(view)
      view.invalidate
    end

    def resume(view)
      set_status
      view.invalidate
    end

    def suspend(_view); end

    def onCancel(_reason, view)
      @stage = :place_origin
      @origin = nil
      @sim.clear
      update_ui
      view.invalidate
      set_status
    end

    # ---- Mouse -----------------------------------------------------------

    def onMouseMove(_flags, x, y, view)
      @mouse_m = ground_point_m(view, x, y)
      view.invalidate if @stage == :place_heading
    end

    def onLButtonDown(_flags, x, y, view)
      pt = ground_point_m(view, x, y)
      return unless pt

      case @stage
      when :place_origin
        @origin = pt
        @stage = :place_heading
      when :place_heading
        heading = heading_to(@origin, pt)
        @sim.place(@origin, heading)
        @stage = :ready
        update_ui
      when :ready
        # Ignore stray clicks; use Esc / panel Reset to re-place.
      end
      view.invalidate
      set_status
    end

    # ---- Keyboard --------------------------------------------------------

    def onKeyDown(key, _repeat, _flags, view)
      return false unless @stage == :ready

      handled = true
      case key
      when VK_UP
        @sim.advance(@sim.step_m)
      when VK_DOWN
        @sim.advance(-@sim.step_m)
      when VK_LEFT
        @sim.steer_by(2.0)
      when VK_RIGHT
        @sim.steer_by(-2.0)
      else
        handled = handle_char_key(key)
      end

      if handled
        view.invalidate
        set_status
      end
      handled
    end

    # ---- Drawing ---------------------------------------------------------

    def draw(view)
      case @stage
      when :place_heading
        draw_placement_preview(view)
      when :ready
        @sim.draw(view)
      end
    end

    def getExtents
      bb = Geom::BoundingBox.new
      if @sim.placed
        @sim.frames.each do |f|
          f[:fp].each do |unit|
            unit[:body].each { |c| bb.add(Util.m_to_pt(c)) }
          end
        end
      end
      bb.add(Util.m_to_pt(@origin)) if @origin
      bb
    end

    private

    def handle_char_key(key)
      # Space bar centres the steering (32 is the space character code).
      if key == 32 || key == 0x20
        @sim.steer_deg = 0.0
        true
      else
        false
      end
    end

    def draw_placement_preview(view)
      return unless @origin

      target = @mouse_m || [@origin[0] + 1.0, @origin[1]]
      heading = heading_to(@origin, target)
      @sim.draw_preview(view, @origin, heading)
    end

    def heading_to(a, b)
      Math.atan2(b[1] - a[1], b[0] - a[0])
    end

    def ground_point_m(view, x, y)
      ray = view.pickray(x, y)
      pt = Geom.intersect_line_plane(ray, [ORIGIN, Z_AXIS])
      pt && Util.pt_to_m(pt)
    end

    def set_status
      msg =
        case @stage
        when :place_origin
          'Click to place the vehicle start point.'
        when :place_heading
          'Move to aim, then click to set the initial heading.'
        else
          'Arrows: Up/Down drive, Left/Right steer. Space centres, Esc resets.'
        end
      Sketchup.set_status_text(msg)
      App.refresh_dialog
    end

    def update_ui
      App.refresh_dialog
    end
  end
end
