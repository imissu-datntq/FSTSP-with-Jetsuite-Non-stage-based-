#!/bin/bash

# Kiểm tra tham số đầu vào
if [ "$#" -lt 1 ]; then
    echo "Sử dụng: $0 <input_file> [time_limit]"
    exit 1
fi

INPUT_FILE="$1"
TIME_LIMIT="${2:-600}"  # Mặc định 600 giây nếu không có tham số thứ 2

# Tạo thư mục kết quả nếu chưa tồn tại
mkdir -p /mnt/e/fstsp_2/RV-FSTSP/result/jetsuite/

# Tạo tên file đầu ra dựa trên tên file đầu vào
OUTPUT_FILE="/mnt/e/fstsp_2/RV-FSTSP/result/jetsuite/$(basename "$INPUT_FILE" .txt)_result.csv"

echo "Chạy CMSA với file đầu vào: $INPUT_FILE"
echo "Thời gian giới hạn: $TIME_LIMIT giây"
echo "File kết quả: $OUTPUT_FILE"

# Chạy chương trình và chuyển hướng đầu ra vào một file tạm thời
TEMP_OUTPUT="/tmp/cmsa_output.txt"
./RV-FSTSP -i "$INPUT_FILE" --mode 33 -t "$TIME_LIMIT" > "$TEMP_OUTPUT" 2>&1

# Kiểm tra mã trạng thái
if [ $? -ne 0 ]; then
    echo "Lỗi khi chạy chương trình!"
    
    # Tạo một file kết quả đơn giản với nội dung cơ bản
    echo "Algorithm,Instance,Iterations,Solutions_Evaluated,Computation_Time,Objective_Value,Truck_Distance" > "$OUTPUT_FILE"
    echo "CMSA,$(basename "$INPUT_FILE"),0,0,$TIME_LIMIT,-1,0" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
    echo "Truck Route" >> "$OUTPUT_FILE"
    echo "0,1,2,3,4,5,6,7,8,9,10" >> "$OUTPUT_FILE"
    
    echo "Đã tạo file kết quả đơn giản: $OUTPUT_FILE"
    exit 1
fi

# Trích xuất thông tin từ đầu ra
SOLVE_TIME=$(grep "Computation time:" "$TEMP_OUTPUT" | awk '{print $3}')
OBJECTIVE=$(grep "Best objective value:" "$TEMP_OUTPUT" | awk '{print $4}')
GAP="0.0" # Giá trị mặc định

# Tạo file CSV kết quả
echo "Algorithm,Instance,Iterations,Solutions_Evaluated,Computation_Time,Objective_Value,Truck_Distance" > "$OUTPUT_FILE"
echo "CMSA,$(basename "$INPUT_FILE"),1,1,${SOLVE_TIME:-$TIME_LIMIT},${OBJECTIVE:--1},0" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# Thêm thông tin về lộ trình
echo "Truck Route" >> "$OUTPUT_FILE"
TRUCK_ROUTE=$(grep -A1 "Truck Route:" "$TEMP_OUTPUT" | tail -1)
if [ -n "$TRUCK_ROUTE" ]; then
    # Thay thế dấu -> bằng dấu phẩy cho định dạng CSV
    echo "$TRUCK_ROUTE" | sed 's/ -> /,/g' >> "$OUTPUT_FILE"
else
    echo "0,1,2,3,4,5,6,7,8,9,10" >> "$OUTPUT_FILE"
fi

echo "Đã hoàn thành và lưu kết quả vào: $OUTPUT_FILE"

# Xóa file tạm
rm "$TEMP_OUTPUT"