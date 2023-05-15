from pyproj import Transformer

# 좌표계 변환을 위한 transformer 객체 생성
utm5179_to_utm32652 = Transformer.from_crs("EPSG:5179", "EPSG:4326")

# 원본 파일 읽기
with open("/home/park/ISEV_2023/src/path_maker/path/final_path_first.txt", "r") as f:
    lines = f.readlines()

# 변환된 좌표를 저장할 리스트 생성
new_lines = []

# 각 라인마다 좌표계 변환을 수행하여 리스트에 저장
for line in lines:
    x, y, z, m = line.strip().split("\t")
    new_x, new_y, new_z, new_m = utm5179_to_utm32652.transform(float(x), float(y), float(z), float(m))
    new_line = f"{new_x}\t{new_y}\t{new_z}\t{new_m}\n"
    new_lines.append(new_line)

# 변환된 좌표를 파일에 저장
with open("/home/park/ISEV_2023/src/path_maker/path/final_path_first_z52.txt", "w") as f:
    f.writelines(new_lines)