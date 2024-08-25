import csv
import xml.etree.ElementTree as ET
from pyproj import Proj
import mgrs

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

# CSVファイルのパス
input_files = {
    'short_left': '../csv/origin/short_left.csv',
    'long_left': '../csv/origin/long_left.csv',
    'short_right': '../csv/origin/short_right.csv',
    'long_right': '../csv/origin/long_right.csv',
    'short_centerline': '../csv/origin/short_center.csv',
    'long_centerline': '../csv/origin/long_center.csv'
}

# OSMファイルのパス
osm_file = '../osm/lanelet2_map.osm'

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