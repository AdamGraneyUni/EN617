typedef enum {
  EMEGERGENCY_STOP,
  PAUSE,
  FAILED_TO_PICKUP_BLOCK,
  RESUME,
  RESET,
  START,
  STOP,
  READY_TO_PICKUP_PAD1,
  PICKED_UP_PAD1,
  READY_TO_DEPOSIT_CONVEYOR,
  READY_TO_PICKUP_CONVEYOR,
  READY_TO_DEPOSIT_PAD2,
  PICKED_UP_CONVEYOR
} can_messages;