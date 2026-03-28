#!/bin/bash

# ============================================================
#  collect_results.sh
#  Đọc toàn bộ file CSV trong folder output và gộp vào một
#  file CSV tổng hợp, mỗi dòng = một bộ dữ liệu.
#
#  Usage:
#    ./collect_results.sh [output_dir] [result_csv]
#
#  Defaults:
#    output_dir  = output
#    result_csv  = output/all_results.csv
# ============================================================

export LC_NUMERIC=C

# ---------- Tham số ----------
OUTPUT_DIR="${1:-output}"
RESULT_CSV="${2:-${OUTPUT_DIR}/all_results.csv}"

# ---------- Kiểm tra thư mục ----------
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Lỗi: Thư mục '$OUTPUT_DIR' không tồn tại."
    exit 1
fi

# ---------- Header ----------
echo "Instance,Objective,Lower Bound,Gap(%),Solve Time(s),Truck Served,Drone Served,Jetsuite Served" \
    > "$RESULT_CSV"

# ---------- Đếm ----------
count=0
skipped=0

# ---------- Tạo danh sách file đã sort theo prefix + maxradius tăng dần ----------
sorted_csv_files=$(find "$OUTPUT_DIR" -maxdepth 1 -name "*.csv" -type f | while IFS= read -r csv_file; do
    base_name=$(basename "$csv_file")
    if [[ "$base_name" =~ ^(.+)-maxradius-([0-9]+)(\.txt)?\.csv$ ]]; then
        prefix="${BASH_REMATCH[1]}"
        radius="${BASH_REMATCH[2]}"
        printf "%s\t%06d\t%s\n" "$prefix" "$radius" "$csv_file"
    else
        printf "%s\t%06d\t%s\n" "$base_name" 999999 "$csv_file"
    fi
done | sort -t $'\t' -k1,1 -k2,2n -k3,3 | cut -f3-)

# ---------- Duyệt từng file CSV (bỏ qua file tổng hợp) ----------
while IFS= read -r csv_file; do

    # Bỏ qua chính file output và các file summary
    basename_f=$(basename "$csv_file")
    if [[ "$basename_f" == "all_results.csv" || "$basename_f" == summary* ]]; then
        continue
    fi

    # Lấy tên instance = tên file bỏ .csv
    instance="${basename_f%.csv}"

    # Đọc các trường
    obj=$(grep -m1 -E "^Objective/Upper bound" "$csv_file" | cut -d',' -f2 | tr -d ' \r')
    lb=$(grep -m1 -E "^Lower bound"            "$csv_file" | cut -d',' -f2 | tr -d ' \r')
    gap=$(grep -m1 -E "^Gap"                   "$csv_file" | cut -d',' -f2 | tr -d ' %\r')
    stime=$(grep -m1 -E "^Solving time"        "$csv_file" | cut -d',' -f2 | tr -d ' \r')
    truck=$(grep -m1 -E "^Truck served"        "$csv_file" | cut -d',' -f2 | tr -d ' \r')
    drone=$(grep -m1 -E "^Drone served"        "$csv_file" | cut -d',' -f2 | tr -d ' \r')
    jet=$(grep -m1   -E "^Jetsuite served"     "$csv_file" | cut -d',' -f2 | tr -d ' \r')

    # Bỏ qua nếu không tìm thấy objective
    if [[ -z "$obj" ]]; then
        ((skipped++))
        continue
    fi

    # Ghi một dòng
    echo "${instance},${obj},${lb},${gap},${stime},${truck},${drone},${jet}" >> "$RESULT_CSV"
    ((count++))

done <<< "$sorted_csv_files"

# ---------- Kết quả ----------
echo "============================================"
echo "  Đã xử lý : $count file"
echo "  Bỏ qua   : $skipped file (không có objective)"
echo "  Output   : $RESULT_CSV"
echo "============================================"
