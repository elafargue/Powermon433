/* temp_lerp.cpp - scruss, 2014-10-06 
 
 int temp_lerp(uint8_t x);
 for input x (0..255) returns equivalent °F value
 
 int fudged_f_to_c(int f);
 special conversion factor from °F to °C;
 *** NOT FOR GENERAL USE ***
 
 */


#include <Arduino.h>
#include "temp_lerp.h"

#define TEMP_TABSIZE 22

static coord_t temp_tab[TEMP_TABSIZE] = {
  {   0, -49 },
  {   5, -45 },
  {  10, -42 },
  {  20, -22 },
  {  30,  -7 },
  {  40,   5 },
  {  50,  16 },
  {  70,  34 },
  {  80,  42 },
  {  90,  49 },
  { 100,  57 },
  { 130,  78 },
  { 150,  94 },
  { 152,  96 },
  { 154,  97 },
  { 158, 101 },
  { 160, 102 },
  { 176, 118 },
  { 180, 121 },
  { 184, 126 },
  { 185, 127 },
  { 255, 127 }
};

int temp_lerp(uint8_t x) {
  int i;
  for (i = 0; i < TEMP_TABSIZE - 1; i++) {
    if (temp_tab[i].x <= x && temp_tab[i + 1].x >= x)
      return temp_tab[i].y + (temp_tab[i + 1].y - temp_tab[i].y) * (x - temp_tab[i].x) / (temp_tab[i + 1].x -
        temp_tab[i].x);
  }
  return -125; 			/* should never get here, so return stupid */
}

int fudged_f_to_c(int f) {
  /****************************************************
   * ###       ####  ###########   
   *  ###       ##    ####   ####  THIS IS ** NOT **
   *  ####      ##    ###    ####  GENERAL-PURPOSE °F
   *  ####      #     ###    ####  TO °C CONVERSION
   *  #####     #     ###    ###   CODE !!!
   *  # ####    #     ###   ###  
   *  #  ####  ##     #######      It is fudged to
   *  #   ###  #     ####  ####    reduce specific
   * ##    ### #     ###    ####   quantization error
   * #     #####     ###    ####   in the Powermon433
   * #      ####     ###    ####   code.
   * #       ###     ###    #### 
   * ##       ##     ####   ####    ** DO NOT REUSE **
   * ####       #    ##########    
   ****************************************************/
  return (10 * f - 308) / 18;
}

