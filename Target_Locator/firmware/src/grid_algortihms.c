
#include "grid_algorithms.h"


////////////////////////////////////////////////////////////////////////////////
///
///
///
void printMap(uint8_t worldMap[][MAPDIM])
{
  int i, j;
  for(j = 0; j < MAPDIM; j++){
    for(i = 0; i < MAPDIM; i++){
      //world[i][j] = 40;
      //// fprintf(stdout, "%2i,%2i,%2i ", i, j, world[i][j]);
      // fprintf(stdout, "%3i ", worldMap[i][MAPDIM - j - 1]);
    }
    // fprintf(stdout, "\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
///
///
///
void printRel(uint8_t relGrid[][11])
{
  int i, j;
  for(i = 0; i < GRID_SIZE; i++){
    for(j = 0; j < GRID_SIZE; j++){
      // fprintf(stdout, "%3i, ", relGrid[i][j]);
    }
    // fprintf(stdout, "\n");
    //// fprintf(stdout, "%i, %i, %i, %i\n", test_grid[i][0], test_grid[i][1], test_grid[i][2], test_grid[i][3]);
  }
}
////////////////////////////////////////////////////////////////////////////////
///
///
///
void transmitObstacle(OBSTACLE_t object, uint8_t msgID)
{
  // fprintf(stdout, "TRANSMITTING OBSTACLE\n\n");
  //uint8_t buf[512];
  messageItem_t debug;
  debug.msgSize = sprintf(debug.payload,"{\"RESPONSE\" : { \"SOURCE\" : \"%s \", \"DEST\" : \"%s\", \"ID\" : %i, \"TYPE\" : \"%s\", \"DATA\" : {\"X\": %i, \"Y\": %i, \"LENGTH\": %i, \"WIDTH\": %i, \"THETA\": %i}}}", TARGET_LOCATOR, PATH_FINDER, OBSTACLE_TYPE, msgID, object.x_center, object.y_center, object.length, object.width, object.theta);
  SendMessageForTransmitQ(debug);
  //send to queue
  // fprintf(stdout, buf);
  // fprintf(stdout, "FINISHED TRANSMIT\n\n");
}

////////////////////////////////////////////////////////////////////////////////
///
///
///
void transmitTarget(TARGET_t target, uint8_t msgID)
{
  //uint8_t buf[512];
//  uint16_t length = sprintf(buf, "{response: { source: TargetLocator, Dest: %s, ID: %i, Type: Target, {x: %i, y: %i}}}", "PATH", msgID, target.x_center, target.y_center);
  messageItem_t debug;
  debug.msgSize = sprintf(debug.payload, "{\"RESPONSE\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %i, \"TYPE\" : \"%s\", \"DATA\" : {\"X\": %i, \"Y\": %i}}}", TARGET_LOCATOR, PATH_FINDER, TARGET_TYPE, msgID, target.x_center, target.y_center);
  SendMessageForTransmitQ(debug);
  // fprintf(stdout, buf);
}

////////////////////////////////////////////////////////////////////////////////
///
///
///
void transmitRelative(uint8_t relGrid[][11], uint8_t msgID)
{
    messageItem_t debug;
    debug.msgSize = sprintf(debug.payload,"{\"RESPONSE\" : { \"SOURCE\" : \"%s\", \"DEST\" : \"%s\", \"ID\" : %i, \"TYPE\" : \"%s\", \"DATA\" : [", TARGET_LOCATOR, PATH_FINDER, 0, GRID_TYPE);
  int i, j;
  for(i = 0; i < 11; i++){
    for(j = 0; j < 11; j++){
        debug.msgSize += sprintf(debug.payload+debug.msgSize, "%i, ",relGrid[i][j]);
      // fprintf(stdout, "%3i, ", relGrid[i][j]);
    }
    // fprintf(stdout, "\n");
    //// fprintf(stdout, "%i, %i, %i, %i\n", test_grid[i][0], test_grid[i][1], test_grid[i][2], test_grid[i][3]);
  }
    debug.msgSize += sprintf(debug.payload+debug.msgSize - 2, " ]}}");
    SendMessageForTransmitQ(debug);
//  // fprintf(stdout, "TRANSMITTING RELATIVE DATA\n\n");
//  //uint8_t buf[512];
//  uint16_t length = sprintf(buf, "{response: { source: TargetLocator, Dest: %s, ID: %i, Type: ObstacleData, {[", "SEARCHER", msgID);
//  //// fprintf(stdout, buf);
//  int i, j;
//  for(i = 0; i < GRID_SIZE; i++){
//    for(j = 0; j < GRID_SIZE; j++){
//      //// fprintf(stdout, "CURRENT LENGTH: %i\n", length);
//      //length += sprintf(&buf[length], "%i,", relGrid[i][j]>DETECT_THRESH);
//    }
//  }
//  length += sprintf(&buf[length-1], "]}}}");
//  // fprintf(stdout, "TOTAL MSG LENGTH: %i\n", length);
//  // fprintf(stdout, buf);
//  // fprintf(stdout, "\n\n");
}




////////////////////////////////////////////////////////////////////////////////
///
///
///takes a relative grid and the rover location, and generates an absolute space, that can be overlayed onto the current world
void transposeRelativeToAbsolute(uint8_t relGrid[GRID_SIZE][GRID_SIZE], 
        uint8_t sizeofRel,
        uint8_t x_loc, 
        uint8_t y_loc,
        int16_t theta, 
        GRID_ITEM_t abs_select[][GRID_SIZE])
{
  int i, j;
  float angle = theta*2*M_PI/360;
  float sin_i = sin(angle);
  float cos_i = cos(angle);
  float new_x, new_y;
  for(i = 0; i< sizeofRel;i++){
    for(j = 0; j<sizeofRel; j++){
      //new_x = i*sin_i + j*cos_i;
      //new_y = i*cos_i - j*sin_i;
      new_x = i*cos_i - j*sin_i;
      new_y = i*sin_i + j*cos_i;
      int8_t x_val, y_val;
      x_val = ROUND(new_x);
      y_val = ROUND(new_y);
      //// fprintf(stdout, "rel: %i %i\nx_output: %f vs. %i\ny_output: %f vs. %i\n\n", i, j, new_x, x_val, new_y, y_val);
      abs_select[i][j].x = x_val + x_loc - GRID_SIZE/2;
      abs_select[i][j].y = y_val + y_loc - GRID_SIZE/2;
      abs_select[i][j].value = relGrid[i][j];
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
///
///
///take an absolute map of some portion of the world, and add it onto the existing world
void updateMyWorld(GRID_ITEM_t newGrid[][GRID_SIZE],
        uint8_t worldGrid[][MAPDIM])
{
  // fprintf(stdout, "UPDATING WORLD\n\n");
  int i_count, i, j_count, j;
  for (i_count = 0; i_count < GRID_SIZE; i_count++){
    for (j_count = 0; j_count < GRID_SIZE; j_count++){
      i = newGrid[i_count][j_count].x;
      j = newGrid[i_count][j_count].y;
      if (i > MAPDIM || j > MAPDIM){
        //TODO throw error, out of bounds
        continue;
      }
      worldGrid[i][j] += newGrid[i_count][j_count].value;//TODO decide how to update
      if(worldGrid[i][j] > MIN_OVERFLOW){//overflow sub
        worldGrid[i][j] = 0;
      }
      if(worldGrid[i][j] > MAX_VAL){//overflow sub
        worldGrid[i][j] = MAX_VAL - 10;
      }
    }
  }
  for(i = 0; i < MAPDIM; i++){
    for(j = 0; j < MAPDIM; j++){
      if(worldGrid[i][j] < DETECT_THRESH){
        if((worldGrid[i+1][j] > DETECT_THRESH) &&
            (worldGrid[i-1][j] > DETECT_THRESH) &&
            (worldGrid[i][j+1] > DETECT_THRESH) &&
            (worldGrid[i][j-1] > DETECT_THRESH)){
          worldGrid[i][j] = DETECT_THRESH+THING;
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
///
///
///takes the absolute world and rover location to generate a relative map of some size
void transposeAbsoluteToRelative(uint8_t worldGrid[][MAPDIM],
        uint8_t x_loc, 
        uint8_t y_loc, 
        int16_t theta, 
        uint8_t relGrid[][11],
        uint8_t sizeofRel)
{
     int i, j;
     uint8_t temp_grid[22][22];
  float angle = theta*2*M_PI/360;
  float sin_i = sin(angle);
  float cos_i = cos(angle);
  float new_x, new_y;
  for(i = 0; i< 22;i++){
    for(j = 0; j<22; j++){
      // new_x = i*sin_i + j*cos_i;
      // new_y = i*cos_i - j*sin_i;
      new_x = i*cos_i - j*sin_i;
      new_y = i*sin_i + j*cos_i;
      int8_t x_val, y_val;
      x_val = ROUND(new_x);
      y_val = ROUND(new_y);
      temp_grid[i][j] = worldGrid[x_val+x_loc - 11][y_val+y_loc - 11];
      //// fprintf(stdout, "rel: %i %i\nx_location: %i\ny_location: %i\nValue: %i\n\n", i, j, x_val+x_loc, y_val+y_loc, worldGrid[x_val+x_loc][y_val+y_loc]);
      // abs_select[i][j].x = x_val + x_loc;
      // abs_select[i][j].y = y_val + y_loc;
      // abs_select[i][j].value = relGrid[i][j];
    }
  }
  for(i = 0; i < 11; i++){
      for(j = 0; j < 11; j++){
          relGrid[i][j] = ((worldGrid[2*i][2*j] > INITIAL_VAL) ||
                        (worldGrid[2*i+1][2*j] > INITIAL_VAL) ||
                        (worldGrid[2*i][2*j+1] > INITIAL_VAL) ||
                        (worldGrid[2*i+1][2*j+1] > INITIAL_VAL));
      }
  }
  // fprintf(stdout, "ABOUT TO TRANSMIT RELATIVE\n\n");
  uint8_t msgID = 10;
  printRel(relGrid);
  transmitRelative(relGrid, msgID);
  printRel(relGrid);
}

////////////////////////////////////////////////////////////////////////////////
///
///
///
inline void recursiveSearch(bool tempGrid[][MAPDIM], uint8_t x, uint8_t y, GRID_ITEM_t *LEFT_Item, GRID_ITEM_t *BOT_Item, GRID_ITEM_t *TOP_Item, GRID_ITEM_t *RIGHT_Item)
{
  if(x < LEFT_Item->x || (x == LEFT_Item->x && y < LEFT_Item->y)){//left+bottom
    LEFT_Item->x = x;
    LEFT_Item->y = y;
  }
  // if(x == LEFT_Item->x && y < LEFT_Item->y)
  if(y < BOT_Item->y || (y == BOT_Item->y && x > BOT_Item->x)){//bottom+right
    BOT_Item->x = x;
    BOT_Item->y = y;
  }
  if(y > TOP_Item->y || (y == TOP_Item->y && x < TOP_Item->x)){//top+left
    TOP_Item->x = x;
    TOP_Item->y = y;
  }
  if(x > RIGHT_Item->x || (x == RIGHT_Item->x && y > RIGHT_Item->y)){//top+right
    RIGHT_Item->x = x;
    RIGHT_Item->y = y;
  }
  
  tempGrid[x][y] = 0;
  //// fprintf(stdout, "Object @ %i, %i\n", x, y);
  bool found = 0;
  if(x > 0){
    if(tempGrid[x-1][y]){
      recursiveSearch(tempGrid, x-1, y, LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item);
      found = 1;
    }
  }
  if(x < MAPDIM-1){
    if(tempGrid[x+1][y]){
      recursiveSearch(tempGrid, x+1, y, LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item);
      found = 1;
    }
  }
  if(y > 0){
    if(tempGrid[x][y-1]){
      recursiveSearch(tempGrid, x, y-1, LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item);
      found = 1;
    }
  }
  if(y < MAPDIM-1){
    if(tempGrid[x][y+1]){
      recursiveSearch(tempGrid, x, y+1, LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item);
      found = 1;
    }
  }
}

static uint8_t classGrid[MAPDIM][MAPDIM];
////////////////////////////////////////////////////////////////////////////////
///
///
///scan world grid and identify where there are objects and what they are
void classifyObjects(uint8_t worldGrid[][MAPDIM], uint8_t msgID)
{
    int i, j;
    uint8_t counter = 1;
    bool temp;
    uint8_t tempArray[256];
    for(i = 0; i < 256; i++){
        tempArray[i] = i;
    }
    for(i = 0; i < MAPDIM; i++){
        for(j = 0; j < MAPDIM; j++){
            classGrid[i][j] = 0;
        }
    }
  for(i = 0; i < MAPDIM; i++){
    for(j = 0; j < MAPDIM; j++){
      temp = ((worldGrid[i][j] > DETECT_THRESH) && (
                    ((i > 0) && (worldGrid[i-1][j] > DETECT_THRESH)) ||
                    ((i < MAPDIM) && (worldGrid[i+1][j] > DETECT_THRESH)) ||
                    ((j > 0) && (worldGrid[i][j-1] > DETECT_THRESH)) ||
                    ((j < MAPDIM) && (worldGrid[i][j+1] > DETECT_THRESH))));
      if(temp){
          if(i > 0){
            if(classGrid[i-1][j]){
                classGrid[i][j] = classGrid[i-1][j];
            }
          }
          if(i < MAPDIM-1){
            if(classGrid[i+1][j]){
                if(classGrid[i][j] > 0 && classGrid[i][j] != classGrid[i+1][j]){
                    //correlate the two
                    if(classGrid[i+1][j] > classGrid[i][j]){
                        tempArray[classGrid[i+1][j]] = classGrid[i][j];
                    }
                    else{
                        tempArray[classGrid[i][j]] = classGrid[i+1][j];
                    }
                }
                else{
                    classGrid[i][j] = classGrid[i+1][j];
                }
            }
          }
          if(j > 0){
            if(classGrid[i][j-1]){
                if(classGrid[i][j] > 0 && classGrid[i][j] != classGrid[i][j-1]){
                    //correlate the two
                    if(classGrid[i][j-1] > classGrid[i][j]){
                        tempArray[classGrid[i][j-1]] = classGrid[i][j];
                    }
                    else{
                        tempArray[classGrid[i][j]] = classGrid[i][j-1];
                    }
                }
                else{
                    classGrid[i][j] = classGrid[i][j-1];
                }
            }
          }
          if(j < MAPDIM-1){
            if(classGrid[i][j+1]){
                if(classGrid[i][j] > 0 && classGrid[i][j] != classGrid[i][j+1]){
                    //correlate the two
                    if(classGrid[i][j+1] > classGrid[i][j]){
                        tempArray[classGrid[i][j+1]] = classGrid[i][j];
                    }
                    else{
                        tempArray[classGrid[i][j]] = classGrid[i][j+1];
                    }
                }
                else{
                    classGrid[i][j] = classGrid[i][j+1];
                }
            }
          }
          if(classGrid[i][j] == 0){
              classGrid[i][j] = counter;
              counter++;
          }
      }
      else{
          classGrid[i][j] = 0;
      }
    }
  }
    for (i = 0; i < 256; i++){
        if(tempArray[i] != tempArray[tempArray[i]]){
            tempArray[i] = tempArray[tempArray[i]];
        }
    }
    for(i = 0; i < MAPDIM; i++){
        for(j = 0; j < MAPDIM; j++){
            classGrid[i][j] = tempArray[classGrid[i][j]];
        }
    }
    int k;
    GRID_ITEM_t LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item;
    for(k = 1; k < 256; k++){
        if (tempArray[k] != k){
            continue;
        }
        LEFT_Item.x = MAPDIM;
        LEFT_Item.y = MAPDIM;
        BOT_Item.x = 0;
        BOT_Item.y = MAPDIM;
        TOP_Item.x = MAPDIM;
        TOP_Item.y = 0;
        RIGHT_Item.x = 0;
        RIGHT_Item.y = 0;
        for(i = 0; i < MAPDIM; i++){
            for(j = 0; j < MAPDIM; j++){
                if(classGrid[i][j] != tempArray[k]){
                    continue;
                }
                //select corners
                if(i < LEFT_Item.x || (i == LEFT_Item.x && j < LEFT_Item.y)){//left+bottom
                    LEFT_Item.x = i;
                    LEFT_Item.y = j;
                }
                // if(x == LEFT_Item.x && y < LEFT_Item.y)
                if(j < BOT_Item.y || (j == BOT_Item.y && i > BOT_Item.x)){//bottom+right
                    BOT_Item.x = i;
                    BOT_Item.y = j;
                }
                if(j > TOP_Item.y || (j == TOP_Item.y && i < TOP_Item.x)){//top+left
                    TOP_Item.x = i;
                    TOP_Item.y = j;
                }
                if(i > RIGHT_Item.x || (i == RIGHT_Item.x && j > RIGHT_Item.y)){//top+right
                    RIGHT_Item.x = i;
                    RIGHT_Item.y = j;
                }
            }
        }
        float angle1 = (360.0/(2*M_PI))*atan(((float)(RIGHT_Item.y - BOT_Item.y))/((float)(RIGHT_Item.x - BOT_Item.x)));
        float angle2 = (360.0/(2*M_PI))*atan(((float)(TOP_Item.y - LEFT_Item.y))/((float)(TOP_Item.x - LEFT_Item.x)));
        int8_t angle = ROUND(((angle1+angle2)/2));
        //// fprintf(stdout, "Angle1: %f\nAngle2: %f\n\n", angle1, angle2);
        // fprintf(stdout, "Final Angle: %i\n\n", angle);
        uint8_t length, width;
        float length1 = sqrt(((float)pow(RIGHT_Item.y - BOT_Item.y, 2))+((float)pow(RIGHT_Item.x - BOT_Item.x, 2)));
        float length2 = sqrt(((float)pow(TOP_Item.y - LEFT_Item.y, 2))+((float)pow(TOP_Item.x - LEFT_Item.x, 2)));
        length = ROUND((((length1+length2) - 1)/2 + 1));
        //// fprintf(stdout, "Length1: %f\nLength2: %f\n\n", length1, length2);
        // fprintf(stdout, "Final Length: %i\n\n", length);
        float width1 = sqrt(((float)pow(TOP_Item.y - RIGHT_Item.y, 2))+((float)pow(TOP_Item.x - RIGHT_Item.x, 2)));
        float width2 = sqrt(((float)pow(LEFT_Item.y - BOT_Item.y, 2))+((float)pow(LEFT_Item.x - BOT_Item.x, 2)));
        width = ROUND((((width1+width2) - 1)/2 + 1));
        //// fprintf(stdout, "Width1: %f\nWidth2: %f\n\n", width1, width2);
        // fprintf(stdout, "Final Width: %i\n\n", width);
        //float angle = atan(((float)(RIGHT_Item.y - BOT_Item.y))/((float)(RIGHT_Item.x - BOT_Item.x)));
        //TODO process 4 corners to make box
        if(length < 6 || width < 6){
          //not an obstacle
          if(length >= 4 && length <= 6 && width >= 4 && width <= 6){
            //target
            TARGET_t new_target;
            new_target.x_center = (RIGHT_Item.x - LEFT_Item.x)/2 + LEFT_Item.x;
            new_target.y_center = (TOP_Item.y - BOT_Item.y)/2 + BOT_Item.y;
            transmitTarget(new_target, msgID);
          }
        }
        else{
          OBSTACLE_t new_obstacle;
          new_obstacle.x_center = (RIGHT_Item.x - LEFT_Item.x)/2 + LEFT_Item.x;;
          new_obstacle.y_center = (TOP_Item.y - BOT_Item.y)/2 + BOT_Item.y;
          new_obstacle.theta = angle;
          new_obstacle.length = length;
          new_obstacle.width = width;
          transmitObstacle(new_obstacle, msgID);
        }
    }
    
}
void classifyObjects2(uint8_t worldGrid[][MAPDIM], uint8_t msgID)
{
  bool tempGrid[MAPDIM][MAPDIM];
  int i, j;
  for(i = 0; i < MAPDIM; i++){
    for(j = 0; j < MAPDIM; j++){
      tempGrid[i][j] = (worldGrid[i][j] > DETECT_THRESH);
      classGrid[i][j] = worldGrid[i][j];
    }
  }
  
  for(i = 0; i < MAPDIM; i++){
    for(j = 0; j < MAPDIM; j++){
      if(tempGrid[i][j]){
        // fprintf(stdout, "CLASSIFYING\n\n");
        GRID_ITEM_t LEFT_Item, BOT_Item, TOP_Item, RIGHT_Item;
        LEFT_Item.x = MAPDIM;
        LEFT_Item.y = MAPDIM;
        BOT_Item.x = 0;
        BOT_Item.y = MAPDIM;
        TOP_Item.x = MAPDIM;
        TOP_Item.y = 0;
        RIGHT_Item.x = 0;
        RIGHT_Item.y = 0;
        recursiveSearch(tempGrid, i, j, &LEFT_Item, &BOT_Item, &TOP_Item, &RIGHT_Item);
        // fprintf(stdout, "Left: %i, %i\nBottom: %i, %i\nTop: %i, %i\nRight: %i, %i\n\n", LEFT_Item.x, LEFT_Item.y, BOT_Item.x, BOT_Item.y, TOP_Item.x, TOP_Item.y, RIGHT_Item.x, RIGHT_Item.y);
        float angle1 = (360.0/(2*M_PI))*atan(((float)(RIGHT_Item.y - BOT_Item.y))/((float)(RIGHT_Item.x - BOT_Item.x)));
        float angle2 = (360.0/(2*M_PI))*atan(((float)(TOP_Item.y - LEFT_Item.y))/((float)(TOP_Item.x - LEFT_Item.x)));
        int8_t angle = ROUND(((angle1+angle2)/2));
        //// fprintf(stdout, "Angle1: %f\nAngle2: %f\n\n", angle1, angle2);
        // fprintf(stdout, "Final Angle: %i\n\n", angle);
        uint8_t length, width;
        float length1 = sqrt(((float)pow(RIGHT_Item.y - BOT_Item.y, 2))+((float)pow(RIGHT_Item.x - BOT_Item.x, 2)));
        float length2 = sqrt(((float)pow(TOP_Item.y - LEFT_Item.y, 2))+((float)pow(TOP_Item.x - LEFT_Item.x, 2)));
        length = ROUND((((length1+length2) - 1)/2 + 1));
        //// fprintf(stdout, "Length1: %f\nLength2: %f\n\n", length1, length2);
        // fprintf(stdout, "Final Length: %i\n\n", length);
        float width1 = sqrt(((float)pow(TOP_Item.y - RIGHT_Item.y, 2))+((float)pow(TOP_Item.x - RIGHT_Item.x, 2)));
        float width2 = sqrt(((float)pow(LEFT_Item.y - BOT_Item.y, 2))+((float)pow(LEFT_Item.x - BOT_Item.x, 2)));
        width = ROUND((((width1+width2) - 1)/2 + 1));
        //// fprintf(stdout, "Width1: %f\nWidth2: %f\n\n", width1, width2);
        // fprintf(stdout, "Final Width: %i\n\n", width);
        //float angle = atan(((float)(RIGHT_Item.y - BOT_Item.y))/((float)(RIGHT_Item.x - BOT_Item.x)));
        //TODO process 4 corners to make box
        if(length < 6 || width < 6){
          //not an obstacle
          if(length >= 4 && length <= 6 && width >= 4 && width <= 6){
            //target
            TARGET_t new_target;
            new_target.x_center = (RIGHT_Item.x - LEFT_Item.x)/2 + LEFT_Item.x;
            new_target.y_center = (TOP_Item.y - BOT_Item.y)/2 + BOT_Item.y;
            transmitTarget(new_target, msgID);
          }
        }
        else{
          OBSTACLE_t new_obstacle;
          new_obstacle.x_center = (RIGHT_Item.x - LEFT_Item.x)/2 + LEFT_Item.x;;
          new_obstacle.y_center = (TOP_Item.y - BOT_Item.y)/2 + BOT_Item.y;
          new_obstacle.theta = angle;
          new_obstacle.length = length;
          new_obstacle.width = width;
          transmitObstacle(new_obstacle, msgID);
        }
      }
    }
  }
    //TODO find where threshold met, identify collections
    //determine size of collections
    //if more than target/vertex
    //check where other robot should be
    //if robot, update pathfinder location
    //if not, classify obstacle, x, y, l, w, theta
}

