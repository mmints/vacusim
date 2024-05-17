/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  This supervisor tracks down the absolute position of the robot
 *               and removes the dust from the area covered by the robot.
 */

/*
 * Modified by Mark O. Mints (mmints@uni-koblenz.de)
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <webots/display.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 64

#define X 0
#define Y 1
#define Z 2

// size of the ground
#define GROUND_X 14.9
#define GROUND_Y 9.9

// Evaluation
#define EVALUATION_TIME 1800.0 // thirty minutes

#define MAP_HEIGHT 512
#define MAP_WIDTH 768

void fillMap(int map[MAP_HEIGHT][MAP_WIDTH], const int i, const int j, const float radius)
{
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            int dx = x - j;
            int dy = y - i;
            double distance = sqrt(dx * dx + dy * dy);

            if (distance <= radius) {
                map[y][x] = 255;
            }
        }
    }
}

int countCleanElements(int map[MAP_HEIGHT][MAP_WIDTH]) {
    int count = 0;

    for (int i = 0; i < MAP_HEIGHT; i++) {
        for (int j = 0; j < MAP_WIDTH; j++) {
            if (map[i][j] == 255) {
                count++;
            }
        }
    }

    return count;
}

// main function
int main() {

    FILE *file;
       
    // Open the file in write mode
    file = fopen("result.csv", "w");
    
    if (file == NULL) {
        printf("Error opening the file.\n");
        return 1;
    }
    
    // Header
    fprintf(file, "time;cleaning_progress\n");

    
  // init Webtos stuff
  wb_robot_init();

  // First we get a handler to devices
  WbDeviceTag display = wb_robot_get_device("large_ground_display");

  // get the properties of the Display
  int width = wb_display_get_width(display);    // 768
  int height = wb_display_get_height(display);  // 512


      FILE *map_file;
    map_file = fopen("result_map.pgm", "w");
    fprintf(map_file, "P2\n768 512\n255\n");

    // Map for logging the driven route and to calculate the cleaned area
    // col, row 
    int map[height][width];
    int dirty_elements = 0;
    int cleaned_elements = 0;

    // Initialize Map
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            if ((i <= 13) || (j <= 13) || (i >= 755)|| (j >= 499) // Outlines
                || (i >= 230 && i <= 244 && j >= 346) || (i >= 230 && i <= 244 && j <= 180) // left wall
                || (i >= 460 && i <= 474 && j >= 346) || (i >= 460 && i <= 474 && j <= 180) // right wall
                || (j >= 166 && j <= 180 && i <= 474) // top wall
                || (j >= 332 && j <= 346 && i <= 474) // bottom wall

            ) {
                // Doors
                if (((j >= 166 && j <= 180 && i >= 90) && (j >= 166 && j <= 180 && i <= 180)) || ((j >= 166 && j <= 180 && i >= 307) && (j >= 166 && j <= 180 && i <= 410)) || ((j >= 332 && j <= 346 && i >= 90) && (j >= 332 && j <= 346 && i <= 180)) || ((j >= 332 && j <= 346 && i >= 307) && (j >= 332 && j <= 346 && i <= 410))) {
                    map[j][i] = 128;
                    dirty_elements++;            
                }
                else {
                    map[j][i] = 0;

                }
            } 
            else {
                map[j][i] = 128;
                dirty_elements++;
            }
        }
    }

  // prepare stuff to get the Robot(IROBOT_CREATE).translation field
  WbNodeRef mybot = wb_supervisor_node_get_from_def("CUSTOM_ROOMBA");
  WbFieldRef translationField = wb_supervisor_node_get_field(mybot, "translation");

  // set the background (otherwise an empty ground is displayed at this step)
  WbImageRef background = wb_display_image_load(display, "dust.jpg");
  wb_display_image_paste(display, background, 0, 0, false);

  // set the pen to remove the texture
  wb_display_set_alpha(display, 0.0);

  // For timing
  int last_casted_time = 0;
  float cleaning_progress = 0.0;
    
  while (wb_robot_step(TIME_STEP) != -1) {

    // Update the translation field
    const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);
    const double transform_x = width * (translation[X] + GROUND_X / 2) / GROUND_X;
    const double transform_y = height * (-translation[Y] + GROUND_Y / 2) / GROUND_Y;
  
    // printf("Current Robot Position: (%i,%i)\n", (int)round(transform_x), (int)round(transform_y));

    // display the robot position
    wb_display_fill_oval(display, transform_x, transform_y, 7, 7);
   
   fillMap(map, (int)round(transform_y), (int)round(transform_x), 7.0);
    cleaned_elements = countCleanElements(map);
    cleaning_progress = (float)cleaned_elements / (float)dirty_elements;
  
    const double simulation_time = wb_robot_get_time();
        
    if (((int)simulation_time % 5 == 0) && ((int)simulation_time != last_casted_time)) {
      last_casted_time = (int)simulation_time;
      printf("Current Time: %i Seconds - Cleaning Progress: %f%%\n", last_casted_time, (cleaning_progress * 100.0));
      fprintf(file, "%i;%f\n", last_casted_time, cleaning_progress);
    }
    
    if (simulation_time >= EVALUATION_TIME) {
      printf("Timeout!\n");
      break;
    }
  }
  printf("Experiment is done!\n");
  
    // Printing the map for verification
    for (int i = 0; i < 512; i++) {
        for (int j = 0; j < 768; j++) {
            fprintf(map_file, "%d ", map[i][j]); // MAP [512][768]
        }
        fprintf(map_file, "\n");
    }

    fclose(map_file);
  
  // Close the file
  fclose(file);
  printf("File created successfully.\n");

  wb_robot_cleanup();

  return 0;
}
