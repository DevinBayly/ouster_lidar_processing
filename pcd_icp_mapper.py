import open3d as o3d
import os
import numpy as np
import copy

from pathlib import Path


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh
def prepare_dataset(pths,voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    source = o3d.io.read_point_cloud(str(pths[0]))
    target = o3d.io.read_point_cloud(str(pths[1]))
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def draw_registration_result(source, target, transformation):
    if os.environ.get("DEBUG",None):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0, 0])
        target_temp.paint_uniform_color([0, 1, 0])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp],
                                          zoom=0.4459,
                                          front=[0.9288, -0.2951, -0.2242],
                                          lookat=[1.6784, 2.0612, 1.4451],
                                          up=[-0.3402, -0.9189, -0.1996])


def makepcloud(pcd_name):
    with open(pcd_name,"rb") as phile:
        line = phile.readline()
        header = []
        while not b"DATA" in line:
            line = phile.readline()
            print(line)
            header.append(line)
        buf = phile.read()
    parts = [[e for e in d.decode().strip().split(" ")[1:]] for d in header[1:] ]
    parts = [p for p in parts if len(p) >0]
    parts
    print("parts are",parts)
    collection = []
    for attri,e in enumerate(parts[3]):
        print("e is ",e)
        for i in range(int(e)):
            name = parts[0][attri]
            if name =="_":
                name+=str(i)+str(attri)
            unit = (parts[2][attri]+parts[1][attri]).lower()
            collection.append((name,unit))
    d = np.dtype(collection)
    d.itemsize
    arr = np.frombuffer(buf[:48*262144],dtype=d)
    # In[40]:
    pc = o3d.geometry.PointCloud()
    # In[52]:
    sub = np.column_stack([arr["x"],arr["y"],arr["z"]])
    # In[53]:
    pc.points = o3d.utility.Vector3dVector(sub)
    # In[55]:
    attr  ="reflectivity"
    maxintensity = np.max(arr[attr])
    colors = np.column_stack([arr[attr]/maxintensity,np.zeros_like(arr[attr]),np.zeros_like(arr[attr])])
    print(colors)
    hist,edges = np.histogram(colors[:,1],bins=100)
    print(hist,edges)
    # In[56]:
    pc.colors = o3d.utility.Vector3dVector(colors)
    # In[57]:
    #o3d.visualization.draw_geometries([pc])
    # In[58]:
    #o3d.io.write_point_cloud("converted.pcd",pc)
    return [pc,arr]


voxel_size = 0.05  # means 5cm for this dataset
src_pth=Path("./135.694091710(1).pcd")
src_goods = makepcloud(src_pth)
trgt_pth=Path("./135.694091710.pcd")
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset([src_pth,trgt_pth],
    voxel_size)
result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)

threshold = 0.02
trans_init =result_ransac.transformation
draw_registration_result(source, target, trans_init)
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)
source.transform(reg_p2p.transformation)
o3d.io.write_point_cloud("changed.pcd",source)
transformed_points = np.asarray(source.points)
src_arr = src_goods[1]

src_arr["x"] = transformed_points[:,0]
src_arr["y"] = transformed_points[:,1]
src_arr["z"] = transformed_points[:,2]
np.save("adjusted.npy",src_arr)
