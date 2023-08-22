# specify the directory containing the bag files
# dir="/home/gari/mani_qp_ws/data/bags"

# #loop through all .bag files and convert in the directory
# for bag_file in $dir/*.bag; do
#     csv_file="${bag_file%.bag}.csv"
#     # specify the topic name as /xxxx
#     rostopic echo -b "$bag_file" -p /franka_state_controller/joint_states > "$csv_file"
#     echo "Converted $bag_file to $csv_file"
# done

# rostopic echo -b joint_position_traj.bag -p /franka_state_controller/joint_position > joint_position_guid.csv
# rostopic echo -b joint_velocity_traj.bag -p /franka_state_controller/joint_velocity > joint_velocity_guid.csv

rostopic echo -b joint_position_real.bag -p /franka_state_controller/joint_position > joint_position_real.csv
rostopic echo -b joint_velocity_real.bag -p /franka_state_controller/joint_velocity > joint_velocity_real.csv
