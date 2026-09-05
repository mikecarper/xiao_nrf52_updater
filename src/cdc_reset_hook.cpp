#include <stdint.h>

#include "usb_msc.h"

// TinyUSB declares tud_cdc_line_state_cb() with its weak attribute. A
// definition in a file that includes TinyUSB therefore remains weak and may
// lose to Adafruit's immediate-reset implementation depending on link order.
// Keep this translation unit free of TinyUSB headers so this definition is
// strong and reliably intercepts the 1200-baud DTR drop.
extern "C" void tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts) {
  (void)rts;
  if (instance == 0 && !dtr) {
    usb_msc::note_cdc_dtr_drop();
  }
}
