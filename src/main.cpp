#include "../include/common.h"
#include "ekfslam.h"
#include "../include/plotter.h"
#include "../include/sensor_info.h"
#include "../include/mapper.h"
#include<iomanip>


using namespace std;

//check input arguemnts:
void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instruction: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/world.dat path/to/sensor.dat";
  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "need an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments (need 2 arguemnts).\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

//main function:
int main(int arc, char* argv[])
{

    check_arguments(arc, argv);

    string in_map_name = argv[1];
    string in_sensor_name = argv[2];

    //read the ground truth map data for all landmarks
    Mapper mapper;
    mapper.initialize(in_map_name);

    //read the measurements with odometry and radar data
    MeasurementPackage measurements;
    measurements.initialize(in_sensor_name);
    // cout << measurements.data.size() << endl;
    Draw draw;

    EKFSLAM ekfslam(mapper.data.size(), 3, 0.01);
    // ekfslam(mapper.data.size());
    //iterate dataset and feed them into EKFSLAM
    for (unsigned int i = 0; i < measurements.data.size(); i++) {
        const auto& record = measurements.data[i];
        draw.Clear();

        ekfslam.Prediction(record.odo);
        ekfslam.Correction(record.scans);
        //------------------------------for debug:
        // for (int matrix = 0; matrix < ekfslam.getMu().size(); matrix++) {
        //   std::cout << ekfslam.getMu()(matrix) << '\n';}
        std::cout << i << "running" << '\n';
        ////////////////////////for debug:
        // if (i == 330) {
        //   draw.Plot_State(ekfslam.getMu(), ekfslam.getSigma(), mapper, ekfslam.getobservedLandmarks(), record.scans);
        //   draw.Show();
        //   //------------------------------for debug:
        //   // for (int matrix = 0; matrix < ekfslam.getMu().size(); matrix++) {
        //   //   std::cout << ekfslam.getMu()(matrix) << '\n';}
        //
        //   }

        draw.Plot_State(ekfslam.getMu(), ekfslam.getSigma(), mapper, ekfslam.getobservedLandmarks(), record.scans);
        // draw.Show();
        // draw.Pause();
        draw.Save("/workspace/images/" + std::to_string(i) + ".png");
}
// draw.Show();
}
