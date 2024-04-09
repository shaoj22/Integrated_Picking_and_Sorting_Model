
#%% 分割训练/测试数据集
import sys
sys.path.append('..')
from generate_instances import Integrated_Instance
from generate_instances.generate_instances import generate_medium_instances
import numpy as np
import os

instance_params_list = generate_medium_instances()

np.random.seed(0)
np.random.shuffle(instance_params_list)

train_ratio = 0.8
train_size = int(len(instance_params_list) * train_ratio)
train_list = instance_params_list[:train_size]
test_list = instance_params_list[train_size:]

cur_dir = os.path.dirname(__file__)
with open(cur_dir + '/data/train_instances.txt', 'w') as f:
    for instance in train_list:
        f.write(str(instance) + '\n')
        
with open(cur_dir + '/data/test_instances.txt', 'w') as f:
    for instance in test_list:
        f.write(str(instance) + '\n')
# %%
