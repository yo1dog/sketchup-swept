# frozen_string_literal: true
#
# main.rb — Entry point loaded by the extension. Wires up the module, menus,
# and the shared application state (App).

require 'json'
require 'sketchup.rb'

require_relative 'presets'
require_relative 'vehicle'
require_relative 'simulation'
require_relative 'tool'
require_relative 'dialog'

module Swept
  # App holds the singletons shared between the tool, the dialog, and the menu.
  module App
    module_function

    def sim
      @sim ||= Simulation.new
    end

    def dialog
      @dialog ||= Dialog.build
    end

    def tool
      @tool
    end

    # Open the control panel and activate the interactive tool.
    def start
      dialog.show unless dialog.visible?
      reactivate_tool
    end

    def reactivate_tool
      @tool = SweptPathTool.new
      Sketchup.active_model.select_tool(@tool)
    end

    # Push the latest status into the dialog (safe if it's closed).
    def refresh_dialog
      Dialog.push_status(@dialog) if @dialog
    end
  end

  unless defined?(@ui_built) && @ui_built
    menu = UI.menu('Extensions').add_submenu('Vehicle Swept Path')
    menu.add_item('Open Control Panel') { App.start }
    menu.add_item('Reset Path') { App.sim.reset; App.refresh_dialog; Dialog.invalidate }
    menu.add_item('Commit to Model') { App.sim.commit(Sketchup.active_model) }

    UI.add_context_menu_handler do |context_menu|
      if App.tool
        context_menu.add_item('Swept Path: Reset') do
          App.sim.reset
          App.refresh_dialog
          Dialog.invalidate
        end
      end
    end

    @ui_built = true
  end
end
