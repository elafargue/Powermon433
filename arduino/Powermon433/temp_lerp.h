#ifndef __TEMP_LERP_H__
#define __TEMP_LERP_H__

typedef struct {
  uint8_t x;
  int8_t y;
} coord_t;

int fudged_f_to_c(int f);
int temp_lerp(uint8_t x);

#endif  //__TEMP_LERP_H__
