# frozen_string_literal: true
#
# swept_path.rb — SketchUp extension loader for "Vehicle Swept Path".
#
# Place this file (and the accompanying `swept_path/` folder) in your SketchUp
# Plugins folder. SketchUp scans the Plugins folder for top-level .rb files and
# runs them at startup; this one registers the extension so it appears under
# Window > Extension Manager and can be enabled/disabled there.

require 'sketchup.rb'
require 'extensions.rb'

module Swept
  PLUGIN_ROOT = File.dirname(__FILE__)

  unless defined?(@loaded) && @loaded
    ext = SketchupExtension.new(
      'Vehicle Swept Path',
      File.join(PLUGIN_ROOT, 'swept_path', 'main')
    )
    ext.description = 'Interactively simulate and visualize vehicle swept ' \
      'paths (turning envelopes and tire tracks) for road-layout design on ' \
      'a flat 2D plane. Steer with arrow keys or the control panel.'
    ext.version = '1.0.0'
    ext.copyright = "© #{Time.now.year}"
    ext.creator = 'Swept Path'

    Sketchup.register_extension(ext, true)
    @loaded = true
  end
end
