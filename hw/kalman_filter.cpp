#include "kalman_filter.h"
// Function that uses IMU data to calculate robot pose, and adapt map based on robot pose
extern "C"{    
void kalman_update(MapCell &cell, pose_t &p_meas, pose_t &var_meas) {
    #pragma HLS INLINE

    // Calculate Kalman gain
    pose_t kalman_gain = cell.variance / (cell.variance + var_meas );

    // If the map is being initialized, take the first measurement as start height
    // variance would not be negative. used it to distinguish empty map
    if(cell.variance == -1){
        cell.height = p_meas;
        cell.variance = var_meas;
    } else {
        // Update the height estimate
        cell.height = cell.height + kalman_gain * (p_meas - cell.height);
        // Update the variance
        cell.variance = (1 - kalman_gain) * cell.variance;
    }
}


// Process point cloud and update map
void kalman_filter(point3d *point_cloud, MapCell *map, idx num_points) {
    #pragma HLS INTERFACE m_axi port=map offset=slave bundle=gmem depth=1024
    #pragma HLS INTERFACE m_axi port=point_cloud offset=slave bundle=gmem depth=1024
    #pragma HLS INTERFACE s_axilite port=return bundle=control
    #pragma HLS INTERFACE s_axilite port=num_points bundle=control
    #pragma HLS INTERFACE s_axilite port=map bundle=control
    #pragma HLS INTERFACE s_axilite port=point_cloud bundle=control

    // Local copy of the map
    MapCell local_map[MAP_HEIGHT * MAP_WIDTH];
    #pragma HLS BIND_STORAGE variable=local_map type=RAM_2P impl=BRAM

    // Initialize local_map from the input map
    for (idx i = 0; i < MAP_HEIGHT * MAP_WIDTH; i++) {
        #pragma HLS PIPELINE off
        local_map[i].height = map[i].height;
        local_map[i].variance = map[i].variance;
    }

    // Process each point in the point cloud
    for (idx i = 0; i < num_points; i++) {
        point3d point = point_cloud[i];

        // Calculate cell indices
        idx cell_x = hls::ceil(point.x * CELL_SIZE);
        idx cell_y = hls::ceil(point.y * CELL_SIZE);

        // Check bounds
        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {      
            // Calculate variance
            pose_t J_s = hls::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            pose_t var_meas = sensor_noise_variance * (J_s * J_s);

            // Calculate 1D index for the local_map
            idx cell_idx = cell_x * MAP_WIDTH + cell_y;

            // std::cout<< point.z <<"\t";
            // Perform Kalman update
            kalman_update(local_map[cell_idx], point.z, var_meas);
        }
    }

    // Write updated local_map back to the global map
    for (idx i = 0; i < MAP_HEIGHT * MAP_WIDTH; i++) {
        #pragma HLS PIPELINE off
        map[i].height = local_map[i].height;
        map[i].variance = local_map[i].variance;
    }
}
}