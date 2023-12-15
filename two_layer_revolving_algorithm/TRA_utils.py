'''
File: TRA_utils.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
some utils for the TRA algorithm, such as get the variable from the solved model.
----------
Author: 626
Created Date: 2023.12.14
'''


import numpy as np


def get_variable_from_solved_model(Variable, update_variable_list, model):
    """ get the variable value from the input gurobi model 
    Args:
        Variable (class): model all variable value
        update_variable_list (list): the variable need to update in this model
        model (gurobi): gurobi model class
    Return:
        Variable: updated Variable
        is_solved: if there has solved variable
    """
    # 初始化variable dict
    variable_dict = {}
    # 是否有解
    is_solved = True
    if model.status == 2: # 有解
        # 获取x
        if 'x' in update_variable_list: # 若x在则更新x
            x_update = np.zeros(Variable.x.shape)
            for i in range(Variable.x.shape[0]):
                for j in range(Variable.x.shape[1]):
                    x_update[i,j] = model.getVarByName(f'x[{i},{j}]').x 
            variable_dict['x'] = x_update
        # 获取y
        if 'y' in update_variable_list: # 若y在则更新y
            y_update = np.zeros(Variable.y.shape)
            for i in range(Variable.y.shape[0]):
                for j in range(Variable.y.shape[1]):
                    y_update[i,j] = model.getVarByName(f'y[{i},{j}]').x
            variable_dict['y'] = y_update
        # 获取z
        if 'z' in update_variable_list: # 若z在则更新z
            z_update = np.zeros(Variable.z.shape)
            for i in range(Variable.z.shape[0]):
                for j in range(Variable.z.shape[1]):
                    z_update[i,j] = model.getVarByName(f'z[{i},{j}]').x
            variable_dict['z'] = z_update
        # 获取a1
        if 'a1' in update_variable_list: # 若a1在则更新a1
            a1_update = np.zeros(Variable.a1.shape)
            for i in range(Variable.a1.shape[0]):
                for j in range(Variable.a1.shape[1]):
                    a1_update[i,j] = model.getVarByName(f'a1[{i},{j}]').x
            variable_dict['a1'] = a1_update
        # 获取b1
        if 'b1' in update_variable_list: # 若b1在则更新b1
            b1_update = np.zeros(Variable.b1.shape)
            for i in range(Variable.b1.shape[0]):
                for j in range(Variable.b1.shape[1]):
                    b1_update[i,j] = model.getVarByName(f'b1[{i},{j}]').x
            variable_dict['b1'] = b1_update
        # 获取c1
        if 'c1' in update_variable_list: # 若c1在则更新c1
            c1_update = np.zeros(Variable.c1.shape)
            for i in range(Variable.c1.shape[0]):
                for j in range(Variable.c1.shape[1]):
                    c1_update[i,j] = model.getVarByName(f'c1[{i},{j}]').x
            variable_dict['c1'] = c1_update
        # 获取d1
        if 'd1' in update_variable_list: # 若d1在则更新d1
            d1_update = np.zeros(Variable.d1.shape)
            for i in range(Variable.d1.shape[0]):
                for j in range(Variable.d1.shape[1]):
                    d1_update[i,j] = model.getVarByName(f'd1[{i},{j}]').x
            variable_dict['d1'] = d1_update
        # 获取c1
        if 'Q' in update_variable_list: # 若Q在则更新Q
            Q_update = np.zeros(Variable.Q.shape)
            for i in range(Variable.Q.shape[0]):
                Q_update[i] = model.getVarByName(f'Q[{i}]').x
            variable_dict['Q'] = Q_update
        # 获取passX
        if 'passX' in update_variable_list: # 若passX在则更新passX
            passX_update = np.zeros(Variable.passX.shape)
            for i in range(Variable.passX.shape[0]):
                for j in range(Variable.passX.shape[1]):
                    passX_update[i,j] = model.getVarByName(f'passX[{i},{j}]').x
            variable_dict['passX'] = passX_update
        # 获取f
        if 'f' in update_variable_list: # 若f在则更新f
            f_update = np.zeros(Variable.f.shape)
            for i in range(Variable.f.shape[0]):
                for j in range(Variable.f.shape[1]):
                    for k in range(Variable.f.shape[2]):
                        f_update[i,j,k] = model.getVarByName(f'f[{i},{j},{k}]').x
            variable_dict['f'] = f_update
        # 获取tos
        if 'tos' in update_variable_list: # 若tos在则更新tos
            tos_update = np.zeros(Variable.tos.shape)
            for i in range(Variable.tos.shape[0]):
                tos_update[i] = model.getVarByName(f'tos[{i}]').x
            variable_dict['tos'] = tos_update
        # 获取toe
        if 'toe' in update_variable_list: # 若toe在则更新toe
            toe_update = np.zeros(Variable.toe.shape)
            for i in range(Variable.toe.shape[0]):
                toe_update[i] = model.getVarByName(f'toe[{i}]').x
            variable_dict['toe'] = toe_update
        # 获取I
        if 'I' in update_variable_list: # 若I在则更新I
            I_update = np.zeros(Variable.I.shape)
            for i in range(Variable.I.shape[0]):
                I_update[i] = model.getVarByName(f'I[{i}]').x
            variable_dict['I'] = I_update
        # 获取Ta
        if 'Ta' in update_variable_list: # 若Ta在则更新Ta
            Ta_update = np.zeros(Variable.Ta.shape)
            for i in range(Variable.Ta.shape[0]):
                for j in range(Variable.Ta.shape[1]):
                    Ta_update[i,j] = model.getVarByName(f'Ta[{i},{j}]').x
            variable_dict['Ta'] = Ta_update
        # 获取Ts
        if 'Ts' in update_variable_list: # 若Ts在则更新Ts
            Ts_update = np.zeros(Variable.Ts.shape)
            for i in range(Variable.Ts.shape[0]):
                for j in range(Variable.Ts.shape[1]):
                    Ts_update[i,j] = model.getVarByName(f'Ts[{i},{j}]').x
            variable_dict['Ts'] = Ts_update
        # 获取Te
        if 'Te' in update_variable_list: # 若Te在则更新Te
            Te_update = np.zeros(Variable.Te.shape)
            for i in range(Variable.Te.shape[0]):
                for j in range(Variable.Te.shape[1]):
                    Te_update[i,j] = model.getVarByName(f'Te[{i},{j}]').x
            variable_dict['Te'] = Te_update
        # 获取T
        if 'T' in update_variable_list: # 若T在则更新T
            T_update = np.zeros(Variable.T.shape)
            for i in range(Variable.T.shape[0]):
                T_update[i] = model.getVarByName(f'T[{i}]').x
            variable_dict['T'] = T_update
        if 'FT' in update_variable_list:
            FT_update = model.getVarByName('FT').x
            variable_dict['FT'] = FT_update
    else: # 无解
        is_solved = False

    return variable_dict, is_solved

