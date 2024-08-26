from PIL import Image
import yaml

# ベースとなる名前（拡張子以外の部分）
base_name = 'modified_occupancy_grid_map'

# 画像ファイルのパスとファイル名
input_image_path = './aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/occupancy_grid_map.pgm'
output_image_path = f'./{base_name}.pgm'

# YAMLファイルのパスとファイル名
input_yaml_path = './aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/map/occupancy_grid_map.yaml'
output_yaml_path = f'./{base_name}.yaml'

# .pgmファイルを開く
image = Image.open(input_image_path)

# 既存の画像サイズを取得
original_width, original_height = image.size

# 上に追加する高さ (ピクセル単位)
up = 10

# 新しい画像の高さ（既存の高さ + 追加する高さ）
new_height = original_height + up

# 新しい画像を作成（ここではモノクロなので「L」モードを使用）
new_image = Image.new('L', (original_width, new_height), color=0)  # 255 は白色、0 は黒色

# 新しい画像に元の画像を貼り付ける（新しい画像の上部に空白を追加）
new_image.paste(image, (0, up))

# 新しい.pgmファイルとして保存
new_image.save(output_image_path)

# .yaml ファイルを読み込む
with open(input_yaml_path, 'r') as file:
    config = yaml.safe_load(file)

# .yaml の origin 値を修正
# up の値と resolution を使用して新しい Y 座標を計算
resolution = config['resolution']
origin_y = config['origin'][1]
new_origin_y = origin_y - (up * resolution)

# 修正した Y 座標を設定
config['origin'][1] = new_origin_y

# 新しい画像パスを設定
config['image'] = f'{base_name}.pgm'

# 新しい .yaml ファイルとして保存
with open(output_yaml_path, 'w') as file:
    yaml.dump(config, file, default_flow_style=False)

print(f"New image saved to {output_image_path}")
print("New origin y-coordinate:", new_origin_y)
print(f"Updated YAML saved to {output_yaml_path}")
