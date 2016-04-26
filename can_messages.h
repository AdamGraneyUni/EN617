typedef enum {
  EMERGENCY_STOP,
  PAUSE,
  RESUME,
  RESET,
  START,
  STOP,
  READY_TO_PICKUP_PAD1,
  PICKED_UP_PAD1,
  READY_TO_PICKUP_CONVEYOR,
  PICKED_UP_CONVEYOR,
  PAD2_CLEAR,
  QUERY_PAD2_STATUS,
  OUTPUT_ROBOT_FINISHED,
  READY,
  ALL_COMPONENTS_READY
} can_messages;