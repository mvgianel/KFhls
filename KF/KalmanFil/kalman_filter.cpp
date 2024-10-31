#include "kalman_filter.h"

// Function to transform a point from sensor frame to map frame
void transform_point_to_map(Point3D sensor_point, Point3D &map_point, const Point3D &sensor_to_map_translation) {
    // TODO    
    map_point.x = sensor_point.x + sensor_to_map_translation.x;
    map_point.y = sensor_point.y + sensor_to_map_translation.y;
    map_point.z = sensor_point.z + sensor_to_map_translation.z;
}

// Function to calculate measurement variance for a point (based on Section III-B)
dat_t calculate_measurement_variance(const Point3D &point, const noise_t sensor_noise_variance) {
    // TODO  
    // Define Jacobians
    dat_t J_s = hls::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

    return sensor_noise_variance * (J_s * J_s);  // Propagate variance using the Jacobian
}

// Kalman Filter update for map cell height estimation
void kalman_update(MapCell &cell, pose_t p_meas, noise_t var_meas) {
    // Calculate Kalman gain
    noise_t kalman_gain = cell.variance / (cell.variance + var_meas);

    // Update the height estimate
    cell.height = cell.height + kalman_gain * (p_meas - cell.height);

    // Update the variance
    cell.variance = (1 - kalman_gain) * cell.variance;
}

// Process point cloud and update map
void process_pointcloud(hls::stream<Point3D> &point_cloud, const Point3D &sensor_to_map_translation, 
noise_t sensor_noise_variance, MapCell map[MAP_WIDTH][MAP_HEIGHT]) {
	Point3D sensor_point;

	while (!point_cloud.empty()) {

    	// Read a point from the point cloud stream
        sensor_point = point_cloud.read();

        // Transform point from sensor frame to map frame
        Point3D map_point;
        transform_point_to_map(sensor_point, map_point, sensor_to_map_translation);

        // Map the 3D map point to a 2D grid cell (x, y) and height measurement (z)
        int cell_x = static_cast<int>(map_point.x / CELL_SIZE);
        int cell_y = static_cast<int>(map_point.y / CELL_SIZE);

        // Ensure cell indices are within bounds
        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT) {
            pose_t p_meas = map_point.z;   // Use the z-coordinate as the height measurement

            // Calculate measurement variance for this point
            noise_t var_meas = calculate_measurement_variance(map_point, sensor_noise_variance);

            // Apply Kalman filter to update height and variance of the map cell
            // pose_t h_prior = map[cell_x][cell_y].height;
            // noise_t var_prior = map[cell_x][cell_y].variance;
            // pose_t h_post; noise_t var_post;

            kalman_update(map[cell_x][cell_y], p_meas, var_meas);

            // Store updated height and variance back to the map
            // map[cell_x][cell_y].height = h_post;
            // map[cell_x][cell_y].variance = var_post;
        }
    }
}
