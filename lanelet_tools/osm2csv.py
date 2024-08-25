import xml.etree.ElementTree as ET
import csv
import os

# OSMファイルのパス
osm_file = os.path.expanduser('../aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/lanelet2_map.osm')

# 出力するCSVファイルのパス
output_files = {
    'short_left': '../csv/origin/short_left.csv',
    'long_left': '../csv/origin/long_left.csv',
    'short_right': '../csv/origin/short_right.csv',
    'long_right': '../csv/origin/long_right.csv',
    'short_center': '../csv/origin/short_center.csv',
    'long_center': '../csv/origin/long_center.csv'
}

# OSMファイルを解析
try:
    tree = ET.parse(osm_file)
    root = tree.getroot()
except ET.ParseError as e:
    print(f"Error parsing OSM file: {e}")
    exit(1)

def read_node(way_ref):
    coordinates = []
    way = root.find(f"way[@id='{way_ref}']")
    if way is not None:
        for nd in way.findall('nd'):
            ref = nd.attrib['ref']
            node = root.find(f"node[@id='{ref}']")
            tags = {tag.attrib['k']: tag.attrib['v'] for tag in node.findall('tag')}
            coordinate = [tags.get('local_x', None), tags.get('local_y', None), 43.1]
            coordinates.append(coordinate)
    else:
        print(f"Way with ID {way_ref} not found in OSM data")

    return coordinates

def read_osm():
    my_list = []
    for relation in root.findall("relation"):
        my_dict = {}
        for member in relation.findall("member"):
            role = member.attrib.get('role')
            ref = member.attrib.get('ref')

            coordinates = read_node(ref)
            if role == 'centerline':
                my_dict["center"] = coordinates
            elif role == 'left':
                my_dict["left"] = coordinates
            elif role == 'right':
                my_dict["right"] = coordinates

        my_list.append(my_dict)

    short_index = 1 if len(my_list[0]['center']) - len(my_list[1]['center']) > 0 else 0

    for index, lanelet in enumerate(my_list):
        name = 'short' if index == short_index else 'long'
        for key, value in lanelet.items():
            file_name = output_files[f"{name}_{key}"]
            with open(file_name, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['local_x', 'local_y', 'local_z'])
                for coord in value:
                    writer.writerow(coord)

def main():
    read_osm()

if __name__ == '__main__':
    main()
