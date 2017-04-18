RAS maze distance sensor 
========================

This node simulates 6 IR distance sensors placed on the kobuki robot. It generates distances (reported as adc values) by checking intersection of the IR beam with the map publishes by the `ras_maze_map` node. 

## Sensor frames

* Frame: `distance_sensor_front_left_link`; reported on adc channel 1.
* Frame: `distance_sensor_back_left_link`; reported on adc channel 2. 
* Frame: `distance_sensor_front_right_link`; reported on adc channel 3. 
* Frame: `distance_sensor_back_right_link`; reported on adc channel 4. 
* Frame: `distance_sensor_forward_right_link`; reported on adc channel 5.
* Frame: `distance_sensor_forward_left_link`; reported on adc channel 6. 

## Topics

The sensor values are published on the topic `/kobuki/adc`.

To visualize the sensor beams in rviz, add a `MarkerArray` on topic `/dist_sensor_markers`. 
