#include "elevator_driver/data_collection.hpp"
#include "elevator_driver/elevator_driver.h"

#include  <signal.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

using namespace elevator_driver;
using std::to_string;
using namespace cv;

int main(int argc, char *argv[]) {
  char* portIn = NULL;
  if (argc < 2) {
    printf("missing command line argument: port");
    return 0;
  }
  portIn = argv[1];

  float height = 0;
  float pressure = 0;

  ElevatorDriver elevatorDriver;

  if (elevatorDriver.connect(portIn) < 0) {
    fprintf(stderr, "Unable to connect to designated deice %s\n", portIn);
    return 0;
  }

    {
        // ElevatorTestManager will flush cached data points when floor switching is over
        // we don't log when switching floors, to avoid duplicate datapoints.
        ElevatorTestManager etm("height_test_report.csv", {"height (m)", "pressure (pa)"});
        while (etm.should_run()) {
            cv::imshow("State Board", *(etm.get_current_state_img())); 
            int keycode = cv::waitKey(100);
            etm.switch_floors(keycode);
            height = elevatorDriver.getHeight();
            pressure = elevatorDriver.getPressure();
            etm.write_data({to_string(height), to_string(pressure)});
            
            printf("\e[2J\e[H");
            printf("Measured Height: %f m\n", height);
            printf("Measured Pressure: %f Pa\n", pressure);
            printf("Current Floor: %d \n", etm.get_current_floor());
            printf("If you are switching floors, hit any key other than the arrow key to stop \n");
        }
        cv::destroyAllWindows(); 	
    }
    system("python3 ../scripts/plot_csv.py --file height_test_report.csv --provide_x");
}