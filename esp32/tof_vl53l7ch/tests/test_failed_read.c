/* Host regression: a failed/partial sensor read must never be decoded. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vl53lmz_api.h"
static uint8_t partial(void *handle, uint16_t reg, uint8_t *data, uint32_t size) {
  (void)handle; (void)reg;
  memset(data, 0xFF, size > 240 ? 240 : size);
  return VL53LMZ_STATUS_ERROR;
}
int main(void) {
  static VL53LMZ_Configuration dev;
  static VL53LMZ_ResultsData output, original;
  memset(&output, 0xA5, sizeof(output));
  memcpy(&original, &output, sizeof(output));
  dev.platform.Read = partial;
  dev.data_read_size = 1444;
  dev.streamcount = 42;
  uint8_t status = vl53lmz_get_ranging_data(&dev, &output);
  assert(status != 0);
  assert(dev.streamcount == 42);
  assert(memcmp(&output, &original, sizeof(output)) == 0);
  puts("PASS: partial read rejected before parsing; output and stream state unchanged");
  return 0;
}
