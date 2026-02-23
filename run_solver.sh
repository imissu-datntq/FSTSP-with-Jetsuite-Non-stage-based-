#!/bin/bash

# Force English locale for consistent number formatting
export LC_NUMERIC=C

input_dir="/media/orlab/437478a6-2666-4549-903d-18e9d7d1c34f/orlab/Quocdat/SLSCM Lab/projects/Niels_instances/output_with_jetsuite/"
output_dir="/media/orlab/437478a6-2666-4549-903d-18e9d7d1c34f/orlab/Quocdat/SLSCM Lab/projects/output_stagebase/"
log_file="$output_dir/run_log.txt"  # File để ghi log từ terminal

# Tạo thư mục output nếu chưa tồn tại
mkdir -p "$output_dir"

# Xóa file log cũ (nếu có)
> "$log_file"

# Tìm các bộ test có n <= 10 (theo regex)
instances=$(ls "$input_dir" | grep -E "^.*n[1-9]-.*$|^.*n1[0-9]-.*$|^.*n10-.*$")

# Đếm tổng số bộ test thỏa mãn điều kiện
total_instances=$(echo "$instances" | wc -l)
echo "Tổng số bộ test (n <= 10): $total_instances" | tee -a "$log_file"

# Biến đếm số lượng bộ test đã giải quyết
solved=0

for instance in $instances; do
  # Tạo đường dẫn đầy đủ đến file input
  input_file="$input_dir/$instance"
  # Tạo đường dẫn đến file kết quả (tên file đầu ra giống tên file đầu vào)
  output_file="$output_dir/${instance%.txt}_result.txt"

  echo "Đang giải quyết bộ test: $instance..." | tee -a "$log_file"
  echo "-----------------------------------" | tee -a "$log_file"

  # Chạy chương trình RV-FSTSP với giới hạn thời gian 1 tiếng (3600 giây)
  timeout 3600 ./build/RV-FSTSP -i "$input_file" -o "$output_dir" | tee "$output_file"

  # Kiểm tra xem chương trình có bị dừng do hết thời gian không
  if [ $? -eq 124 ]; then
    echo "Bộ test $instance đã hết thời gian giải (1 tiếng)." | tee -a "$log_file"
    # Tạo file kết quả thủ công với thông báo hết thời gian
    echo "Timeout: Giải quyết bộ test $instance vượt quá thời gian cho phép (1 tiếng)." > "$output_file"
  else
    echo "Bộ test $instance đã được giải quyết thành công." | tee -a "$log_file"
    # Kiểm tra xem file kết quả có được tạo không
    if [ ! -s "$output_file" ]; then
      echo "Chương trình không tạo file kết quả hoặc file rỗng. Tạo file kết quả thủ công." | tee -a "$log_file"
      echo "Kết quả giải quyết bộ test $instance." > "$output_file"
    fi
  fi

  # Tăng biến đếm
  solved=$((solved + 1))
  echo "Đã giải quyết $solved/$total_instances bộ test." | tee -a "$log_file"
  echo "Kết quả được lưu tại: $output_file" | tee -a "$log_file"
  echo "-----------------------------------" | tee -a "$log_file"
  echo | tee -a "$log_file"
done

echo "Hoàn thành! Tất cả $total_instances bộ test (n <= 10) đã được giải quyết." | tee -a "$log_file"