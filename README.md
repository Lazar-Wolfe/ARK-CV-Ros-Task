# ARK-CV-Ros-Task
Get centroid and length of cube given set of pose, rgb and segmented images.
First run camera_matrix_1.py to start publishing the camera matrix calculated from the pose. Then run the feature_detector_2.py to start publishing information about corners.
Finally run triangulation_3.py to subscribe as both publishers will publish only once.
Currently the cube corners plotted do not project a cube and distances vary from 1-1000. New changes will be updated. 
