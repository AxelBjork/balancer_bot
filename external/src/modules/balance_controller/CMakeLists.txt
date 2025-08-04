px4_add_module(
  MODULE         balance_controller
  MAIN           balance_controller
  SRCS           balance_controller.cpp
  STACK_MAIN     2500          # bytes
)
