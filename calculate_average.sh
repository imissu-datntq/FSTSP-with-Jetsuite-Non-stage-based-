#!/bin/bash

# Force English locale for consistent number formatting
export LC_NUMERIC=C

# Script để tính average objective từ các file CSV trong thư mục output

if [ $# -eq 0 ]; then
    csv_dir="output_stagebase"
else
    csv_dir="$1"
fi

if [ ! -d "$csv_dir" ]; then
    echo "Lỗi: Thư mục '$csv_dir' không tồn tại"
    exit 1
fi

# Xóa file summary cũ nếu tồn tại
summary_file="$csv_dir/summary_statistics.csv"
if [ -f "$summary_file" ]; then
    rm "$summary_file"
    echo "Đã xóa file summary cũ: $summary_file"
fi

echo "Đang phân tích các file CSV trong thư mục: $csv_dir"
echo "================================================================"

# Tìm tất cả file CSV (trừ run_log.txt)
csv_files=$(find "$csv_dir" -name "*.csv" -type f)

if [ -z "$csv_files" ]; then
    echo "Không tìm thấy file CSV nào trong thư mục '$csv_dir'"
    exit 1
fi

# Đếm số file
total_files=$(echo "$csv_files" | wc -l)
echo "Tổng số file CSV: $total_files"
echo ""

# Tính toán statistics
result=$(echo "$csv_files" | while read csv_file; do
    # Tìm các giá trị bằng regex (hỗ trợ nhiều format)
    obj=$(grep -E "^(Objective|Upper bound|Objective/Upper bound)" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    lb=$(grep -E "^Lower bound" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    gap=$(grep -E "^Gap" "$csv_file" | cut -d',' -f2 | tr -d ' %')
    time=$(grep -E "^Solving time" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    truck=$(grep -E "^Truck served" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    drone=$(grep -E "^Drone served" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    jetsuite=$(grep -E "^Jetsuite served" "$csv_file" | cut -d',' -f2 | tr -d ' ')
    
    # Kiểm tra nếu objective là số hợp lệ
    if [[ "$obj" =~ ^[0-9]+\.?[0-9]*$ ]]; then
        echo "$obj $lb $gap $time $truck $drone $jetsuite"
    fi
done | awk '{
    sum_obj += $1
    sum_lb += $2
    sum_gap += $3
    sum_time += $4
    sum_truck += $5
    sum_drone += $6
    sum_jetsuite += $7
    count++
    
    if ($1 < min_obj || NR == 1) min_obj = $1
    if ($1 > max_obj || NR == 1) max_obj = $1
}
END {
    if (count > 0) {
        printf "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d", sum_obj/count, sum_lb/count, sum_gap/count, sum_time/count, min_obj, max_obj, sum_truck/count, sum_drone/count, sum_jetsuite/count, count
    } else {
        print "0 0 0 0 0 0 0 0 0 0"
    }
}')

# Parse kết quả
avg_obj=$(echo "$result" | cut -d' ' -f1)
avg_lb=$(echo "$result" | cut -d' ' -f2)
avg_gap=$(echo "$result" | cut -d' ' -f3)
avg_time=$(echo "$result" | cut -d' ' -f4)
min_obj=$(echo "$result" | cut -d' ' -f5)
max_obj=$(echo "$result" | cut -d' ' -f6)
avg_truck=$(echo "$result" | cut -d' ' -f7)
avg_drone=$(echo "$result" | cut -d' ' -f8)
avg_jetsuite=$(echo "$result" | cut -d' ' -f9)
valid_count=$(echo "$result" | cut -d' ' -f10)

echo "KẾT QUẢ THỐNG KÊ:"
echo "================================================================"
echo "Số instance hợp lệ:        $valid_count / $total_files"
echo ""
echo "OBJECTIVE VALUE:"
echo "  - Average (Trung bình):  $avg_obj"
echo "  - Minimum (Nhỏ nhất):    $min_obj"
echo "  - Maximum (Lớn nhất):    $max_obj"
echo ""
echo "LOWER BOUND (Average):     $avg_lb"
echo "GAP (Average):             $avg_gap%"
echo "SOLVE TIME (Average):      $avg_time giây"
echo "================================================================"

# Tạo file summary CSV
summary_file="$csv_dir/summary_statistics.csv"
echo "Metric,Value" > "$summary_file"
echo "Total CSV Files,$total_files" >> "$summary_file"
echo "Valid Instances,$valid_count" >> "$summary_file"
echo "Average Objective,$avg_obj" >> "$summary_file"
echo "Min Objective,$min_obj" >> "$summary_file"
echo "Max Objective,$max_obj" >> "$summary_file"
echo "Average Lower Bound,$avg_lb" >> "$summary_file"
echo "Average Gap,$avg_gap%" >> "$summary_file"
echo "Average Solve Time,$avg_time" >> "$summary_file"

echo ""
echo "Kết quả đã được lưu vào: $summary_file"
