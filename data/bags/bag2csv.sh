# specify the directory containing the bag files
# dir="/home/gari/mani_qp_ws_for_traj/src/mani_qp_controller/data/bags"

# #loop through all .bag files and convert in the directory
# for bag_file in $dir/*.bag; do
#     csv_file="${bag_file%.bag}.csv"
#     base_filename=$(basename "$bag_file" .bag)
#     # specify the topic name as /xxxx
#     rostopic echo -b "$bag_file" -p /franka_state_controller/joint_position > "$csv_file"
#     # rostopic echo -b "$bag_file" -p /franka_state_controller/joint_position > "$dir/$base_filename.csv"
#     echo "Converted $bag_file to $csv_file"
# done

# # rostopic echo -b joint_position_traj.bag -p /franka_state_controller/joint_position > joint_position_guid.csv
# # rostopic echo -b joint_velocity_traj.bag -p /franka_state_controller/joint_velocity > joint_velocity_guid.csv

rostopic echo -b joint_position_real_0911.bag -p /franka_state_controller/joint_position > joint_position_real_demo_offset.csv
rostopic echo -b joint_velocity_real_0911.bag -p /franka_state_controller/joint_velocity > joint_velocity_real_demo_offset.csv

