#include "main.h"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/draw/lv_draw_rect.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_txt.h"
#include "liblvgl/widgets/lv_btnmatrix.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
// #include "pros/apix.h"
#include <cstddef>

#define LEFT_MOTOR_PORTS {7, 5}
#define RIGHT_MOTOR_PORTS {8, 19}
#define PNEUMATIC_PORT 'a'
#define INTAKE_PORTS {1, 15}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

static void event_handler(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);
  switch (code) {
  case LV_EVENT_VALUE_CHANGED: {
    uint32_t id = lv_btnmatrix_get_selected_btn(obj);
    const char *txt = lv_btnmatrix_get_btn_text(obj, id);
    LV_LOG_USER("%s was pressed\n", txt);
    break;
  }
  case LV_EVENT_DRAW_PART_BEGIN: {
    lv_obj_draw_part_dsc_t *dsc =
        (lv_obj_draw_part_dsc_t *)lv_event_get_param(e);

    dsc->rect_dsc->radius = LV_RADIUS_CIRCLE;
    lv_palette_t palette;
    switch (dsc->id) {
    case 0:
    case 1:
      palette = LV_PALETTE_BLUE;
    case 2:
      palette = LV_PALETTE_PURPLE;
    case 3:
    case 4:
      palette = LV_PALETTE_PURPLE;
    default:
      palette = LV_PALETTE_YELLOW;
    };
    if (lv_btnmatrix_get_selected_btn(obj) == dsc->id)
      dsc->rect_dsc->bg_color = lv_palette_darken(palette, 3);
    else
      dsc->rect_dsc->bg_color = lv_palette_main(palette);
    dsc->label_dsc->color = lv_color_white();
    break;
  }
  default:
    return;
  }
}

lv_obj_t *createBtn(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                    lv_coord_t width, lv_coord_t height, int id,
                    const char *title) {
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, width, height);
  // lv_obj_set_free_num(btn, id);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, title);
  lv_obj_align(label, LV_TEXT_ALIGN_CENTER, 0, 5);

  return btn;
}

static const char *btnm_map[] = {"L", "R", "\n", "SKILLS", "\n", "L", "R"};

void initialize() {
  lv_obj_t *btnm = lv_btnmatrix_create(lv_scr_act());
  lv_obj_add_event_cb(btnm, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_center(btnm);

  // createBtn(lv_scr_act(), 0, 0, 20, 20, 0, "bog");
}

/**
 * Runs while the robot is in the disabled state
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous or opcontrol
 */
void competition_initialize() {}

void autonomous() {}

void auton_LR() {}
void auton_RR() {}
void auton_LB() {}
void auton_RB() {}
void auton_SKILLS() {}

void opcontrol() {
  int32_t DEADZONE = 10;
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_mg(LEFT_MOTOR_PORTS);
  pros::MotorGroup right_mg(RIGHT_MOTOR_PORTS);
  pros::MotorGroup intake_mg(INTAKE_PORTS);
  pros::adi::Pneumatics pneumatic(PNEUMATIC_PORT, 0);
  left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (1) {
    int dir = master.get_analog(
        ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int turn = master.get_analog(
        ANALOG_RIGHT_X); // Gets the turn left/right from right joystick

    // if joystick is in deadzone, it counts as zero
    // prevents unintentional drift
    if (abs(dir) < DEADZONE)
      dir = 0;
    if (abs(turn) < DEADZONE)
      turn = 0;

    if (dir && turn == 0) {
      left_mg.brake();
      right_mg.brake();
    } else {
      left_mg.move(dir - turn);  // Sets left motor voltage
      right_mg.move(dir + turn); // Sets right motor voltage
    }

    if (master.get_digital(DIGITAL_R2))
      intake_mg.move(127);
    if (master.get_digital(DIGITAL_R1))
      intake_mg.move(-127);
    if (master.get_digital(DIGITAL_R1) == master.get_digital(DIGITAL_R2))
      intake_mg.brake();
    if (master.get_digital(DIGITAL_L1))
      pneumatic.extend();
    if (master.get_digital(DIGITAL_L2))
      pneumatic.retract();
    if (master.get_digital(DIGITAL_A))
      pneumatic.set_value(pneumatic.toggle());

    pros::delay(5);
  }
}