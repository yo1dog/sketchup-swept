# frozen_string_literal: true
#
# dialog.rb — The HtmlDialog control panel and its callbacks.

module Swept
  module Dialog
    module_function

    def build
      dlg = UI::HtmlDialog.new(
        dialog_title: 'Vehicle Swept Path',
        preferences_key: 'com.sweptpath.panel',
        scrollable: true,
        resizable: true,
        width: 340,
        height: 620,
        min_width: 300,
        style: UI::HtmlDialog::STYLE_DIALOG
      )
      dlg.set_file(File.join(Swept::PLUGIN_ROOT, 'swept_path', 'html', 'panel.html'))
      register_callbacks(dlg)
      dlg
    end

    def register_callbacks(dlg)
      # Sent by the page once it has loaded — reply with presets + status.
      dlg.add_action_callback('ready') do |_ctx|
        dlg.execute_script("SweptUI.setPresets(#{presets_json})")
        push_status(dlg)
      end

      dlg.add_action_callback('select_preset') do |_ctx, key|
        App.sim.load_preset(key)
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('set_steer') do |_ctx, deg|
        App.sim.steer_deg = deg.to_f
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('steer_by') do |_ctx, deg|
        App.sim.steer_by(deg.to_f)
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('advance') do |_ctx, mult|
        App.sim.advance(App.sim.step_m * mult.to_f)
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('set_step') do |_ctx, m|
        App.sim.step_m = [m.to_f.abs, 0.01].max
        push_status(dlg)
      end

      dlg.add_action_callback('set_ghost') do |_ctx, m|
        App.sim.ghost_spacing_m = [m.to_f.abs, 0.1].max
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('set_options') do |_ctx, json|
        opts = JSON.parse(json)
        App.sim.show_body_traces = opts['body']
        App.sim.show_wheel_tracks = opts['tracks']
        App.sim.show_ghosts = opts['ghosts']
        invalidate
      end

      dlg.add_action_callback('set_projection') do |_ctx, json|
        o = JSON.parse(json)
        App.sim.show_projection_fwd = o['fwd']
        App.sim.show_projection_rev = o['rev']
        App.sim.project_mode = (o['mode'] == 'steps' ? :steps : :distance)
        if App.sim.project_mode == :steps
          App.sim.project_steps = [o['value'].to_i, 1].max
        else
          App.sim.project_distance_m = [o['value'].to_f, 0.1].max
        end
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('reset') do |_ctx|
        App.sim.reset
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('clear') do |_ctx|
        App.sim.clear
        App.reactivate_tool
        push_status(dlg)
        invalidate
      end

      dlg.add_action_callback('commit') do |_ctx|
        App.sim.commit(Sketchup.active_model)
        push_status(dlg)
      end

      dlg.add_action_callback('place') do |_ctx|
        App.reactivate_tool
      end
    end

    def push_status(dlg)
      return unless dlg&.visible?

      dlg.execute_script("SweptUI.setStatus(#{App.sim.status_json})")
    end

    def presets_json
      Presets::LIST.map { |p| { key: p[:key], name: p[:name] } }.to_json
    end

    def invalidate
      view = Sketchup.active_model&.active_view
      view&.invalidate
    end
  end
end
