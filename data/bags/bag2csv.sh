# specify the directory containing the bag files
dir="/home/gari/mani_qp_ws/data/bags"

#loop through all .bag files and convert in the directory
for bag_file in $dir/*.bag; do
    csv_file="${bag_file%.bag}.csv"
    # specify the topic name as /xxxx
    rostopic echo -b "$bag_file" -p /franka_state_controller/joint_states > "$csv_file"
    echo "Converted $bag_file to $csv_file"
done

# rostopic echo -b q_traj.bag -p /franka_state_controller/joint_states > joint_states.csv