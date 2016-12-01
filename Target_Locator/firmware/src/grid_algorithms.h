/* 
 * File:   grid_algorithms.h
 * Author: Daniel
 *
 * Created on November 4, 2016, 4:43 AM
 */

#ifndef GRID_ALGORITHMS_H
#define	GRID_ALGORITHMS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
    
#include "uart_public.h"

#define ROVER_X 15
#define ROVER_Y 15
#define ROVER_THETA 0

#define GRID_SIZE 18
#define MAPDIM 120
#define INITIAL_VAL 80
#define MAX_VAL 230
#define MIN_OVERFLOW 240
#define THING 3
#define EMPTY 254
#define DETECT_THRESH 100
#define NOTHING_THRESH 20
#define ROUND(x) ((x > (floor(x)+0.5f)) ? ceil(x) : floor(x))

typedef struct{
  uint8_t x;
  uint8_t y;
  uint8_t value;
}GRID_ITEM_t;

typedef struct{
  uint8_t x_center;
  uint8_t y_center;
  int16_t theta;
  uint8_t length;
  uint8_t width;
}OBSTACLE_t;

typedef struct{
  uint8_t x_center;
  uint8_t y_center;
}TARGET_t;

typedef struct
{
    uint8_t left;
    uint8_t right;
    uint8_t center;
    uint8_t back;
}PING_DATA_t;


void printMap(uint8_t worldMap[][MAPDIM]);
void printRel(uint8_t relGrid[][11]);
void transmitObstacle(OBSTACLE_t object, uint8_t msgID);
void transmitTarget(TARGET_t target, uint8_t msgID);
void transmitRelative(uint8_t relGrid[][11], uint8_t msgID);
void transposeRelativeToAbsolute(uint8_t relGrid[GRID_SIZE][GRID_SIZE], 
        uint8_t sizeofRel,
        uint8_t x_loc, 
        uint8_t y_loc,
        int16_t theta, 
        GRID_ITEM_t abs_select[][GRID_SIZE]);
void updateMyWorld(GRID_ITEM_t newGrid[][GRID_SIZE],
        uint8_t worldGrid[][MAPDIM]);
void transposeAbsoluteToRelative(uint8_t worldGrid[][MAPDIM],
        uint8_t x_loc, 
        uint8_t y_loc, 
        int16_t theta, 
        uint8_t relGrid[][11],
        uint8_t sizeofRel);
void recursiveSearch(bool tempGrid[][MAPDIM], uint8_t x, uint8_t y, GRID_ITEM_t *LEFT_Item, GRID_ITEM_t *BOT_Item, GRID_ITEM_t *TOP_Item, GRID_ITEM_t *RIGHT_Item);
void classifyObjects(uint8_t worldGrid[][MAPDIM], uint8_t msgID);

#ifdef	__cplusplus
}
#endif

#endif	/* GRID_ALGORITHMS_H */

