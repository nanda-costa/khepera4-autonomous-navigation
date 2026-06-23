#ifndef WEBOTS_STUB_H
#define WEBOTS_STUB_H

#ifdef __has_include
#  if __has_include(<webots/robot.h>)
#    include <webots/robot.h>
#  else
#    warning "Webots header not found; using stub wb_robot_* definitions"
#    define WB_ROBOT_HEADER_MISSING
#  endif
#else
#  include <webots/robot.h>
#endif

#ifdef WB_ROBOT_HEADER_MISSING
static inline void wb_robot_init(void) {}
static inline int wb_robot_step(int ms) { (void)ms; return 0; }
static inline void wb_robot_cleanup(void) {}
#endif

#endif