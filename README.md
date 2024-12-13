# KF in HLS
## Project Description
In my project, I accelerate a recursive algorithm used to estimate a system's state from noisy measurements (Kalman Filter) to update the height of the cells of an elevation map. 

## What is implemented
This repository implements a Kalman Filter, which is part of https://github.com/ANYbotics/elevation_mapping. However, due to time limitations, the implementation was adapted to not use Robot Operating System (ROS). As a result, all capability was not implemented and data cannot be streamed from a device as was envisioned.

### Implemenmted
These additions seeked to reduce resource utilization and latency but their impact has not yet to be measured
+ Quantized implementaion of an elevation map which is populated based on sensor measurements  
  + Map has a defined resolution to account for cm in the measurements
  + Map has height estimate and variance of said estimate
+ Streaming of new pointcloud data 
+ Update of height in the map using kalman filter principles
+ changed floating point operations to use fixed point
+ using HLS friendly ceil to get consistent cell partitions
  + Before the cells were not always what I expected, so changed the way the partitions were made
+ added INLINE, DATAFLOW, and PIPELINE pragmas
These additions were made to allow for communication between the processor, which loads data into the FPGA, and the hardware design, which is in charge of updating the cells of a map stored in the PS
+ changed code to not use streaming as it was creating errors
+ created host file to allow for the communication between FPGA fabric and processor
+ Streaming and non-streaming designs devised with no II violations

### Will not get implemented
+ Interfacing with PS to get sensor data in real-time
+ Since sensor data would need to be streamed to the cloud from my laptop this will not be implemented as it will be inefficient

## Organization and Running
This project was built on Vitis 2023.2 using part xcu280-fsvh2892-2L-e and setting the clock to 250MHz for the streaming and to 150MHz for non-streaming design, and the top function is kalman_filter().
### The vitis project was configured in the following manner:
#### app_component
The app_component generates the application_component excecutable. Can choose strm (streaming) or nonStrm (axi) design. the files needed to compile the app_component are:
Either:
+ host.cpp
+ host.h

Or:
+ host_stream.cpp
+ host_strm.h
#### hls_component
This can be used to build the kernel. The testbench file uses a map file and a pointcloud file to test the created map against the golden_otput.txt file, which has the expected heights of the updated cells

For m_axi kernel:
+ kalman_filter.cpp
+ kalman_filert.h
+ kf_test.cpp              (testbench)
  
For streaming kernel:
+ kalman_filter_strm.cpp
+ kalman_filert_strm.h
+ kf_test_strm.cpp         (testbench)
#### system_project
Uses the app and hls components to generate the .xclbin files for different purposes. 
+ app_component            (excecutable)
+ kalman_filter_hw.xclbin  (hardware target)
+ kalman_filter_hwe.xclbin (hardware emulation target)
+ kalman_filter_swe.xclbin (software emulation target)
  
These can be found in the sw folder, in a zip file. They are only available for non-streaming implementation.
#### Data
These files are used to test the implementation correctness
+ pointcloud_data.txt      (pointcloud data for testing)
+ map1.txt                 (map file for testing)
+ golden_output.txt        (file with expected output)
#### Outputs
+ updated_map1.txt
#### Report
Report draft
+ draft_report_gianello.pdf 

## Running
To run the code run 
```
export XCL_EMULATION_MODE=target
```
where taget can be sw_emu, hw_emu, or hw. Then, the following command can be used 
```
./app_component -x {xclbin_file} {pointcloud_data_file} {initial_map_file} {golden_output_file}
```
Where the arguments in curly brackets can be replaced with whattever file is desired. For example:
```
./app_component -x kalnman_filter_swe.xclbin pointcloud_data.txt map1.txt golden_output.txt
```
Note that all files must be in the same directory or that the full path to the file must be used. Also check to use the xclbin file related to the desired target.

The output of this should be some messages informing of how the code is going and then a message saying TEST PASSED. However, the processor code has some error that are still being fixed, so the full messages do not come up yet.

### Evaluation
Due to time time constraints the implementation was tested with made up data, which can be found in pointcloud_data.txt. This is also used when the application is deployed in the U280.
