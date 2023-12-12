PACKAGE = "autorace_2023"

from rqt_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("hue_white_l",        int_t,      0,      "hue_white_l",        0,  0, 179)
gen.add("hue_white_h",        int_t,      0,      "hue_white_h",        111,  0, 179)
gen.add("saturation_white_l", int_t,      0,      "saturation_white_l", 0,  0, 255)
gen.add("saturation_white_h", int_t,      0,      "saturation_white_h", 38,  0, 255)
gen.add("lightness_white_l",  int_t,      0,      "lightness_white_l",  126,  0, 255)
gen.add("lightness_white_h",  int_t,      0,      "lightness_white_h",  255,  0, 255)

gen.add("hue_yellow_l",       int_t,      0,      "hue_yellow_l",       0,  0, 179)
gen.add("hue_yellow_h",       int_t,      0,      "hue_yellow_h",       127,  0, 179)
gen.add("saturation_yellow_l",int_t,      0,      "saturation_yellow_l",70,  0, 255)
gen.add("saturation_yellow_h",int_t,      0,      "saturation_yellow_h",255,  0, 255)
gen.add("lightness_yellow_l", int_t,      0,      "lightness_yellow_l", 95,  0, 255)
gen.add("lightness_yellow_h", int_t,      0,      "lightness_yellow_h", 255,  0, 255)

exit(gen.generate(PACKAGE, "autorace_2023", "detect_lane_cfg"))