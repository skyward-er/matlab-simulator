# Sensor Analysis

This folder contains all the scripts needed to perform sensor analysis for current and future missions.

The folder is structured as follows:
- `<YYYY>_<RocketName>`
    - `mat_creator.m`
    - `<RocketName>_<MissionShort>_sensor_vect.mat`
    - `<RocketName>_<MissionShort>_sensor_vect_res.mat`
- `Functions`
- `main_noiseAnalysis.m`
- `NoiseCreator.mlx`

Each rocket has a folder named `<YYY>_<RocketName>`, containing a `mat_creator.m` script to generate the `<RocketName>_<MissionShort>_sensor_vect.mat`, which is then fed into `main_noiseAnalysis.m` to create `<RocketName>_<MissionShort>_sensor_vect_res.mat`.

The resulting `.mat` file is used by `initSensors`, and it must be copied into `matlab-simulator/data/<YYYY>_<RocketName>_<Mission>_<Month>`.

# Script Info

- `main_noiseAnalysis.m`

    This is the main script for sensor noise modeling. A specific sensor can be analyzed, and/or the final `.mat` file can be created.

- `mat_creator.m`
    
    This script creates the initial `.mat` file, which contains all the parameters needed to analyze the sensors. These parameters can be tuned in `main_noiseAnalysis.m`, where, in the first part of the script, it is possible to analyze one sensor at a time.

- `NoiseCreator.mlx`

    A simple live script to better understand how filters and parameters affect noise behavior.

The `Functions` folder contains all the necessary functions for the scripts to work properly, ranging from a window viewer for quick checks to different noise type analyzers.

# Main Use Case

1) Gather a log of the sensor to be analyzed and its sampling frequency (there's a quick `fs` analyzer in the `main_noiseAnalysis.m` script; remember to use the `fs` of the log, not that of the sensor).
2) Open `main_noiseAnalysis.m` and load the path to the log(s).
3) In the section `%% 1 - Single sensor test`, all tests can be performed to ensure correct noise modeling. Start by entering the required data and proceed to tweak the unknowns (`noise_type` and `colored_data`). The bounds need to be selected to gather enough data for the noise analysis (there is no fixed rule) and, most importantly, to avoid flight phases or strange dynamics.
4) Use `%% 1.1 - Visualizer` to check the validity of the window.
5) Use `%% 1.2 - Single sensor analysis` to verify the analysis (white noise sensors will output some results; it’s important to understand that a visual approach is needed to validate the outcome).
6) Enter all the correct data into `mat_creator.m`, including all the sensors for the mission.
7) Go to the `mat_creator` folder and run the script to generate the first `.mat` file.
8) Enter the `main_noiseAnalysis.m` folder and run `%% 2 - Multiple sensors analysis`. Adjust the parameters to fit the specific mission.
9) Copy and paste the newly generated `.mat` file into the prescribed folder (refer to the console for details).
10) Update `initSensors` according to the required configuration.
