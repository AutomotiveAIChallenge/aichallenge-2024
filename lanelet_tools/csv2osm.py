import csv
import xml.etree.ElementTree as ET
from pyproj import Proj
import mgrs
import numpy as np

# 基準点のMGRSコード
mgrs_base_code = "54SUE000000"  # グリッド54SUEの左下隅を基準点とする
mgrs_converter = mgrs.MGRS()
origin_lat, origin_lon = mgrs_converter.toLatLon(mgrs_base_code)

# 基準点の緯度経度をUTM座標に変換
utm_proj = Proj(proj="utm", zone=54, datum="WGS84")
origin_utm_x, origin_utm_y = utm_proj(origin_lon, origin_lat)

def calculate_lat_lon_from_local_xy(x, y):
    utm_x = origin_utm_x + float(x)
    utm_y = origin_utm_y + float(y)

    lon, lat = utm_proj(utm_x, utm_y, inverse=True)
    calculated_mgrs_code = mgrs_converter.toMGRS(lat, lon, MGRSPrecision=3)

    return lon, lat, calculated_mgrs_code

def find_closest_point(target, points):
    """ targetに最も近いポイントをpointsから探す """
    target_point = np.array(target)
    points_array = np.array(points)
    distances = np.linalg.norm(points_array - target_point, axis=1)
    closest_index = np.argmin(distances)
    return closest_index

# CSVファイルのパス
input_files = {
    'short_left': './csv/osm_to_csv/short_left.csv',
    'long_left': './csv/osm_to_csv/long_left.csv',
    'short_right': './csv/osm_to_csv/short_right.csv',
    'long_right': './csv/osm_to_csv/long_right.csv',
}

center_path = './csv/csv_to_osm/gray_aichallenge.csv'
short_center = './csv/osm_to_csv/short_center.csv'

with open(short_center, newline='') as file:
    reader = csv.DictReader(file)
    first_row = next(reader)
    short_center_first_point = (float(first_row['local_x']), float(first_row['local_y']))

center_points = []
downsample_interval = 5  # n個ごとに1つのデータポイントを選択

with open(center_path, newline='') as file:
    reader = csv.reader(file)
    next(reader)  # ヘッダーをスキップ
    for i, row in enumerate(reader):
        if i % downsample_interval == 0:
            center_points.append((float(row[0]), float(row[1])))

closest_index = find_closest_point(short_center_first_point, center_points)
closest_point = center_points[closest_index]

upper_points = center_points[:closest_index + 1]
lower_points = center_points[closest_index:]

# 結果を表示
# print("Closest point:", closest_point)
# print("Upper points count:", len(upper_points))
# print("Lower points count:", len(lower_points))

# OSMファイルのパス
osm_file = './osm/lanelet2_map.osm'

root = ET.Element("osm", version="0.6", generator="custom_conversion")

counter = 1

def create_osm_from_csv():
    global counter
    short_lanelet = {}
    long_lanelet = {}
    count = 0
    start_index = 1
    end_index = 1

    for key, value in input_files.items():
        node_refs = []
        with open(value, newline='') as file:
            reader = list(csv.DictReader(file))
            num = len(reader)

            for index, row in enumerate(reader):
                if count % 2 == 1:
                    if index == 0:
                        node_refs.append(str(end_index))
                    elif index == num - 1:
                        node_refs.append(str(start_index))
                    else:
                        local_x = row['local_x']
                        local_y = row['local_y']
                        lon, lat, mgrs_code = calculate_lat_lon_from_local_xy(local_x, local_y)
                        node = ET.SubElement(root, "node", {
                            "id": str(counter),
                            "lat": str(lat),
                            "lon": str(lon)
                        })
                        ET.SubElement(node, "tag", {"k": "local_x", "v": str(local_x)})
                        ET.SubElement(node, "tag", {"k": "local_y", "v": str(local_y)})
                        ET.SubElement(node, "tag", {"k": "ele", "v": str(43.1)})
                        ET.SubElement(node, "tag", {"k": "mgrs_code", "v": str(mgrs_code)})
                        node_refs.append(str(counter))
                        counter += 1
                else:
                    local_x = row['local_x']
                    local_y = row['local_y']
                    lon, lat, mgrs_code = calculate_lat_lon_from_local_xy(local_x, local_y)
                    node = ET.SubElement(root, "node", {
                        "id": str(counter),
                        "lat": str(lat),
                        "lon": str(lon)
                    })
                    ET.SubElement(node, "tag", {"k": "local_x", "v": str(local_x)})
                    ET.SubElement(node, "tag", {"k": "local_y", "v": str(local_y)})
                    ET.SubElement(node, "tag", {"k": "ele", "v": str(43.1)})
                    ET.SubElement(node, "tag", {"k": "mgrs_code", "v": str(mgrs_code)})
                    node_refs.append(str(counter))
                    counter += 1


        way = ET.SubElement(root, "way", {"id": str(counter)})
        for ref in node_refs:
            ET.SubElement(way, "nd", {"ref": ref})
        ET.SubElement(way, "tag", {"k": "type", "v": "line_thin"})
        ET.SubElement(way, "tag", {"k": "subtype", "v": "solid"})

        left_part, right_part = key.split("_")
        if left_part == "short":
            short_lanelet[right_part] = counter
        elif left_part == "long":
            long_lanelet[right_part] = counter

        counter += 1

        if count % 2 == 0: 
            end_index = node_refs[-1]
            start_index = node_refs[0]

        count += 1

    node_refs = []
    for point in upper_points:
        local_x = point[0]
        local_y = point[1]
        lon, lat, mgrs_code = calculate_lat_lon_from_local_xy(local_x, local_y)
        node = ET.SubElement(root, "node", {
                            "id": str(counter),
                            "lat": str(lat),
                            "lon": str(lon)
        })
        ET.SubElement(node, "tag", {"k": "local_x", "v": str(local_x)})
        ET.SubElement(node, "tag", {"k": "local_y", "v": str(local_y)})
        ET.SubElement(node, "tag", {"k": "ele", "v": str(43.1)})
        ET.SubElement(node, "tag", {"k": "mgrs_code", "v": str(mgrs_code)})
        node_refs.append(str(counter))
        counter += 1

    end_index = node_refs[-1]
    start_index = node_refs[0]

    way = ET.SubElement(root, "way", {"id": str(counter)})
    for ref in node_refs:
        ET.SubElement(way, "nd", {"ref": ref})
    ET.SubElement(way, "tag", {"k": "type", "v": "line_thin"})
    ET.SubElement(way, "tag", {"k": "subtype", "v": "solid"})

    long_lanelet['centerline'] = counter

    counter += 1

    node_refs = []
    num = len(lower_points)
    for index, point in enumerate(lower_points):
        if index == 0:
            node_refs.append(str(end_index))
        elif index == num - 1:
            node_refs.append(str(start_index))
        else:
            local_x = point[0]
            local_y = point[1]
            lon, lat, mgrs_code = calculate_lat_lon_from_local_xy(local_x, local_y)
            node = ET.SubElement(root, "node", {
                                "id": str(counter),
                                "lat": str(lat),
                                "lon": str(lon)
            })
            ET.SubElement(node, "tag", {"k": "local_x", "v": str(local_x)})
            ET.SubElement(node, "tag", {"k": "local_y", "v": str(local_y)})
            ET.SubElement(node, "tag", {"k": "ele", "v": str(43.1)})
            ET.SubElement(node, "tag", {"k": "mgrs_code", "v": str(mgrs_code)})
            node_refs.append(str(counter))
            counter += 1

    way = ET.SubElement(root, "way", {"id": str(counter)})
    for ref in node_refs:
        ET.SubElement(way, "nd", {"ref": ref})
    ET.SubElement(way, "tag", {"k": "type", "v": "line_thin"})
    ET.SubElement(way, "tag", {"k": "subtype", "v": "solid"})

    short_lanelet['centerline'] = counter


    short_relation = ET.SubElement(root, "relation", {"id": str(counter)})
    ET.SubElement(short_relation, "tag", {"k": "type", "v": "lanelet"})
    ET.SubElement(short_relation, "tag", {"k": "subtype", "v": "road"})
    ET.SubElement(short_relation, "tag", {"k": "speed_limit", "v": "40"})
    ET.SubElement(short_relation, "tag", {"k": "location", "v": "urban"})
    ET.SubElement(short_relation, "tag", {"k": "one_way", "v": "yes"})

    for key, value in short_lanelet.items():
        ET.SubElement(short_relation, "member", {"type": "way", "role": key, "ref": str(value)})

    counter += 1

    long_relation = ET.SubElement(root, "relation", {"id": str(counter)})
    ET.SubElement(long_relation, "tag", {"k": "type", "v": "lanelet"})
    ET.SubElement(long_relation, "tag", {"k": "subtype", "v": "road"})
    ET.SubElement(long_relation, "tag", {"k": "speed_limit", "v": "40"})
    ET.SubElement(long_relation, "tag", {"k": "location", "v": "urban"})
    ET.SubElement(long_relation, "tag", {"k": "one_way", "v": "yes"})

    for key, value in long_lanelet.items():
        ET.SubElement(long_relation, "member", {"type": "way", "role": key, "ref": str(value)})

    indent(root)
    tree = ET.ElementTree(root)
    tree.write(osm_file, encoding='utf-8', xml_declaration=True)

# インデントと改行を適用する関数
def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for subelem in elem:
            indent(subelem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def main():
    create_osm_from_csv()

if __name__ == '__main__':
    main()