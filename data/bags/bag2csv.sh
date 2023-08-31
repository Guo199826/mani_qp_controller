# specify the directory containing the bag files
dir_pos="/home/gari/mani_qp_ws_for_traj/src/mani_qp_controller/data/bags/pos"
dir_vel="/home/gari/mani_qp_ws_for_traj/src/mani_qp_controller/data/bags/vel"
csv_dir="/home/gari/mani_qp_ws_for_traj/src/mani_qp_controller/data/csv"

# loop through all .bag files in the folder and convert into csv without the title line
# For position.bag:
for bag_file in $dir_pos/*.bag; do
    csv_file="${bag_file%.bag}.csv"
    base_filename=$(basename "$bag_file" .bag)
    # specify the topic name as /xxxx
    rostopic echo -b "$bag_file" -p /franka_state_controller/joint_position > "$csv_dir/$base_filename.csv"
    # rostopic echo -b "$bag_file" -p /franka_state_controller/joint_position > "$dir/$base_filename.csv"
    echo "Converted $bag_file to $base_filename.csv"
    sed -i '1d' "$csv_dir/$base_filename.csv"
done

# For velocity.bag:
for bag_file in $dir_vel/*.bag; do
    csv_file="${bag_file%.bag}.csv"
    base_filename=$(basename "$bag_file" .bag)
    # specify the topic name as /xxxx
    rostopic echo -b "$bag_file" -p /franka_state_controller/joint_velocity > "$csv_dir/$base_filename.csv"
    # rostopic echo -b "$bag_file" -p /franka_state_controller/joint_position > "$dir/$base_filename.csv"
    echo "Converted $bag_file to $csv_dir/$base_filename.csv"
    sed -i '1d' "$csv_dir/$base_filename.csv"
done

# rostopic echo -b joint_position_traj.bag -p /franka_state_controller/joint_position > joint_position_guid.csv
# rostopic echo -b joint_velocity_traj.bag -p /franka_state_controller/joint_velocity > joint_velocity_guid.csv

# rostopic echo -b joint_position_real.bag -p /franka_state_controller/joint_position > joint_position_real.csv
# rostopic echo -b joint_velocity_real.bag -p /franka_state_controller/joint_velocity > joint_velocity_real.csv

