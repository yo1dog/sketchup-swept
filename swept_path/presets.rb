# frozen_string_literal: true
#
# presets.rb — Standard design-vehicle dimensions.
#
# All lengths are in METRES. The kinematics engine works in metres internally
# and only converts to SketchUp's internal units (inches) when drawing.
#
# A vehicle is a *chain* of units:
#   - unit 0 is the LEAD unit (tractor / car), driven with a bicycle model.
#   - each following unit is TOWED, hitched to the unit in front of it.
#
# Unit fields:
#   :kind             :lead or :towed
#   :wheelbase        lead only — rear axle to front (steered) axle
#   :drawbar          towed only — hitch/kingpin to this unit's axle
#   :front_overhang   distance from this unit's reference axle to its front face
#                     (lead: measured from the FRONT axle; towed: from its axle)
#   :rear_overhang    distance from the reference axle to the rear face
#   :width            overall body width
#   :track            wheel track width (defaults to 0.85 * width)
#   :min_turn_radius  lead only — centreline turning radius; sets max steer angle
#   :hitch_offset     distance from this unit's reference axle to the point where
#                     the NEXT unit hitches (positive = forward). Only needed on
#                     units that tow something.
#
module Swept
  module Presets
    LIST = [
      {
        key: 'FORD_TRANSIT_350_XLT',
        name: 'Ford Transit 350 XLT',
        units: [
          { kind: :lead, wheelbase: 3.81, front_overhang: 1.0080625, rear_overhang: 1.2017375,
            width: 2.0621625, min_turn_radius: 6.1 }
        ]
      },
      {
        key: 'P',
        name: 'Passenger car (P)',
        units: [
          { kind: :lead, wheelbase: 3.4, front_overhang: 0.9, rear_overhang: 1.5,
            width: 2.1, min_turn_radius: 7.3 }
        ]
      },
      {
        key: 'SU30',
        name: 'Single-unit truck (SU-30)',
        units: [
          { kind: :lead, wheelbase: 6.1, front_overhang: 1.2, rear_overhang: 1.8,
            width: 2.44, min_turn_radius: 12.8 }
        ]
      },
      {
        key: 'BUS40',
        name: 'City transit bus (BUS-40)',
        units: [
          { kind: :lead, wheelbase: 7.62, front_overhang: 2.1, rear_overhang: 2.6,
            width: 2.6, min_turn_radius: 12.8 }
        ]
      },
      {
        key: 'CAR_TRAILER',
        name: 'Car + utility trailer',
        units: [
          { kind: :lead, wheelbase: 3.4, front_overhang: 0.9, rear_overhang: 1.5,
            width: 2.1, min_turn_radius: 7.3, hitch_offset: -1.2 },
          { kind: :towed, drawbar: 3.5, front_overhang: 3.9, rear_overhang: 0.8,
            width: 2.1 }
        ]
      },
      {
        key: 'WB50',
        name: 'Semi-trailer (WB-50)',
        units: [
          { kind: :lead, wheelbase: 6.1, front_overhang: 1.4, rear_overhang: 0.9,
            width: 2.44, min_turn_radius: 12.8, hitch_offset: 0.6 },
          { kind: :towed, drawbar: 12.5, front_overhang: 13.4, rear_overhang: 1.3,
            width: 2.6 }
        ]
      },
      {
        key: 'WB67D',
        name: 'Double-trailer (WB-67D)',
        units: [
          { kind: :lead, wheelbase: 6.1, front_overhang: 1.4, rear_overhang: 0.9,
            width: 2.44, min_turn_radius: 13.7, hitch_offset: 0.6 },
          { kind: :towed, drawbar: 6.9, front_overhang: 7.8, rear_overhang: 2.4,
            width: 2.6, hitch_offset: -1.5 },
          { kind: :towed, drawbar: 6.9, front_overhang: 7.8, rear_overhang: 1.2,
            width: 2.6 }
        ]
      }
    ].freeze

    DEFAULT = LIST.first

    def self.find(key)
      LIST.find { |p| p[:key] == key } || DEFAULT
    end
  end
end
