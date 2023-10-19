'''
File: clustering_algorithm.py
Project: Integrated_Picking_and_Sorting_Model
Description: Implementing a clustering algorithm for order group batches
File Created: 2023.10.16
Author: 626
'''


from sklearn.cluster import KMeans
from generate_random_data import generate_random_data
import numpy as np

def perform_kmeans_clustering(data, num_clusters):
    """
    使用K均值聚类对订单数据进行聚类。
    
    参数：
    data: 二维列表，每行代表一个订单，每列代表不同的SKU，元素代表SKU的数量。
    num_clusters: 聚类的数量。
    
    返回值：
    聚类结果，包含每个订单所属的簇。
    """
    # 将二维列表转换为NumPy数组
    order_data = np.array(data)
    # 应用K均值聚类算法
    kmeans = KMeans(n_clusters=num_clusters, random_state=42, n_init="auto")
    kmeans.fit(order_data)
    # 获取聚类结果
    cluster_labels = kmeans.labels_
    return cluster_labels


if __name__ == "__main__":
    # 产生随机data
    data = generate_random_data(3, 5, 20, 10)
        # 打印生成的随机订单数据
    print("random data:")
    for i, order in enumerate(data):
        print(f"Order {i + 1}: {order}")
    num_clusters = 2
    cluster_results = perform_kmeans_clustering(data, num_clusters)
    # 打印聚类结果
    for i, cluster_label in enumerate(cluster_results):
        print(f"Order {i + 1} belongs to Cluster {cluster_label + 1}")