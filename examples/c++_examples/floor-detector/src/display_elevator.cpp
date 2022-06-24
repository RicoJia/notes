#include "elevator_driver/data_collection.hpp"
#include "elevator_driver/elevator_driver.h"

using namespace elevator_driver;
using std::to_string;

#include  <signal.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
const float WAIST_HEIGHT = 1.2;
const float SECOND_FLOOR_HEIGHT = 4.7;

bool running = true;
// RJ's waistline is 1.2m, 2nd floor is 4.7m
const int SLEEP_US = 300000;
void  INThandler(int sig)
{
    running = false;
}

// if != 0, then there is data to be read on stdin
int kbhit()
{
    // timeout structure passed into select
    struct timeval tv;
    // fd_set passed into select
    fd_set fds;
    // Set up the timeout.  here we can wait for 1000us second
    tv.tv_sec = 0;
    tv.tv_usec = 1000;

    // Zero out the fd_set - make sure it's pristine
    FD_ZERO(&fds);
    // Set the FD that we want to read
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    // select takes the last file descriptor value + 1 in the fdset to check,
    // the fdset for reads, writes, and errors.  We are only passing in reads.
    // the last parameter is the timeout.  select will return if an FD is ready or 
    // the timeout has occurred
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    // return 0 if STDIN is not ready to be read.
    return FD_ISSET(STDIN_FILENO, &fds);
}



int main(int argc, char *argv[]) {
  char* portIn = NULL;
  if (argc < 2) {
    printf("missing command line argument: port");
    return 0;
  }
  portIn = argv[1];

  float height = 0;
  float pressure = 0;
  float ground_truth_height = WAIST_HEIGHT;

  ElevatorDriver elevatorDriver;


  if (elevatorDriver.connect(portIn) < 0) {
    fprintf(stderr, "Unable to connect to designated deice %s\n", portIn);
    return 0;
  }
    double time_i = 0;
    signal(SIGINT, INThandler);
    {
        ElevatorTestManager etm("height_test_report.csv", {"height (m)", "pressure (pa)"});
        while (running) {
            printf("\e[2J\e[H");
            // ElevatorTestManager will flush cached data points when floor switching is over
            // we don't log when switching floors, to avoid duplicate datapoints.
            if (kbhit()){
                if(getchar() == '\n'){
                    etm.switch_floors();
                }
            }
            else{
                elevatorDriver.readSerial();
                height = elevatorDriver.getHeight();
                pressure = elevatorDriver.getPressure();
                etm.write_data({to_string(height), to_string(pressure)});
                printf("Measured Height: %f m\n", height);
                printf("Measured Pressure: %f Pa\n", pressure);
                printf("Switching Floors? %s \n", etm.switching_floors()? "Yes" : "No");
                printf("Hit enter to signal floor switching; Hit ctrl-c to stop data collection \n");
            }
            usleep(SLEEP_US);
        }

    }

    // {
    //   // Scoping for RAII Object csv_writer
    //   CSVWriter csv_writer("height_test_report.csv");
    //   csv_writer.write_csv_line({"time", "height", "ground_truth_height", "pressure"});
    //   while (running) {
    //     printf("\e[2J\e[H");
    //     if (kbhit()){
    //           if(getchar() == '\n'){
    //               ground_truth_height = (ground_truth_height == SECOND_FLOOR_HEIGHT? WAIST_HEIGHT: SECOND_FLOOR_HEIGHT);
    //           }
    //     }
    //     elevatorDriver.readSerial();
    //     height = elevatorDriver.getHeight();
    //     pressure = elevatorDriver.getPressure();
    //     printf("Measured Height: %f m\n", height);
    //     printf("Measured Pressure: %f Pa\n", pressure);
    //     printf("Ground Truth Height (hit enter to switch): %f m\n", ground_truth_height);
    //     csv_writer.write_csv_line({to_specified_precision_str(time_i*SLEEP_US/1e6, 3), to_string(height), to_string(ground_truth_height), to_string(pressure)});
    //     time_i+=1;
    //     usleep(SLEEP_US);
    //   }
    //
    // }
    system("python3 ../scripts/plot_csv.py --file height_test_report.csv --provide_x");
}
