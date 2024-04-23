# tmp

import numpy as np

def los_to_earth(latitude, longitude, altitude, roll, pitch, yaw):
    # 座標をラジアンに変換
    latitude = np.radians(latitude)
    longitude = np.radians(longitude)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # 地球楕円体のパラメータ
    a = 6378137.0  # 赤道半径
    c = 6356752.314245  # 極半径

    # 位置ベクトルの計算
    x = (a + altitude) * np.cos(latitude) * np.cos(longitude)
    y = (a + altitude) * np.cos(latitude) * np.sin(longitude)
    z = (c + altitude) * np.sin(latitude)

    # 指向ベクトルの計算
    u = np.cos(pitch) * np.sin(yaw)
    v = np.cos(pitch) * np.cos(yaw)
    w = -np.sin(pitch)

    # 二次方程式の係数
    value = -a**2*c**2*w*z - a**2*c**2*v*y - a**2*c**2*u*x
    radical = a**2*c**2*(u**2 + v**2 + w**2) - (c**2*(u*x + v*y)**2 + a**2*(w*z)**2)
    magnitude = a**2*c**2*(u**2 + v**2 + w**2)

    if radical < 0:
        raise ValueError("The Line-of-Sight vector does not intersect with the Earth")
    
    d = (value - np.sqrt(radical)) / magnitude
    intersection_point = np.array([x + d * u, y + d * v, z + d * w])

    # 交点の緯度、経度、高度を計算
    lat = np.arctan2(intersection_point[2], np.sqrt(intersection_point[0]**2 + intersection_point[1]**2))
    lon = np.arctan2(intersection_point[1], intersection_point[0])
    h = np.sqrt(intersection_point[0]**2 + intersection_point[1]**2 + intersection_point[2]**2) - a

    # 交点までの距離
    distance = np.linalg.norm(intersection_point - np.array([x, y, z]))

    return {
        'distance': distance,
        'latitude': np.degrees(lat),
        'longitude': np.degrees(lon),
        'altitude': h
    }

# 使用例
result = los_to_earth(35.0, 139.0, 400000, 0, -45, 90)
print(result)