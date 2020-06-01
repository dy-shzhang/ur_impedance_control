rostopic echo -b staterecord.bag /ur_driver/joint_speed/points[0]/velocities > speed.csv
rostopic echo -b staterecord.bag /joint_speed_record/angle > force.csv
