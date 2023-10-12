from pyproj import Proj,Transformer


# def ll2xy(lat, lon):
#     WGS84 = Proj(init='EPSG:4326')  # WGS84
#     p = Proj(init="EPSG:32650")  # UTM 50N
#
#     # lon = 116.35604 #经度东经，经度范围0-180
#     # lat = 40.00643  #纬度北纬，纬度范围0-90
#     # 对应xy为445036.7208032364 4428669.456629878
#     x, y = transform(WGS84, p, lon, lat)
#     return x,y
def ll2xy(lat, lon):
    transformer = Transformer.from_crs("epsg:4326", "epsg:32650")

    #lon = 116.35604 #经度东经，经度范围0-180
    #lat = 40.00643  #纬度北纬，纬度范围0-90
    # 对应xy为445036.7208032364 4428669.456629878

    x, y = transformer.transform(lat, lon)  # 将WGS84中的lon和lat转换到p坐标下边
    return x, y

if __name__ == "__main__":
    path_lat_lon = [(37, 121), (40.005527, 116.349804), (40.005485, 116.349806), (40.005434, 116.349807),
                    (40.005389, 116.349813), (40.005335, 116.349835), (40.005289, 116.349863), (40.005254, 116.349882),
                    (40.005231, 116.349902), (40.005206, 116.349894), (40.005204, 116.349864), (40.005230, 116.349843),
                    (40.005264, 116.349839), (40.005294, 116.349834)]
    path_x_y = []
    for i in path_lat_lon:
        x, y = ll2xy(i[0], i[1])
        path_x_y.append((x, y))
    print(path_x_y)
