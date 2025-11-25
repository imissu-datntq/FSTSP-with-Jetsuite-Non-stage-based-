#!/bin/bash

# Define the top-level folder path and the binary file path
folder_path="/home/cuong/CLionProjects/RV-FSTSP/generated_instances"
binary_file="/home/cuong/CLionProjects/RV-FSTSP/cmake-build-release/RV-FSTSP"

# Define the values for dtl_values
dtl_values=(20)

# Loop through the dtl_values
for val in "${dtl_values[@]}"; do
    # Iterate through class folders in the folder_path
    find "$folder_path" -mindepth 1 -maxdepth 1 -type d | while IFS= read -r class_folder; do
        # Iterate through instance folders in the class_folder
        find "$class_folder" -mindepth 1 -maxdepth 1 -type d | while IFS= read -r instance_folder; do
            # Extract the folder name from the full path
            instance_name=$(basename "$instance_folder")
            class_name=$(basename "$class_folder")
            output="/home/cuong/CLionProjects/RV-FSTSP/rv_${class_name}/${val}_${instance_name}.out"
            # Create the output file
            touch "$output"

            # Call the binary file with the full instance folder path and the dtl value as arguments
            "$binary_file" "$instance_folder" "$val" 2>&1 | tee "$output"
        done
    done
done