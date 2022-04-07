
import pandas as pd
import numpy as np
import math

def main():
    f = np.fromfile(open("datafile-2.dat"), dtype = np. float32)
    h = f.tolist()
    
    df_X = pd.DataFrame({'pow':h})
    df_X['shifted_pow'] = df_X['pow'].shift(1)
    df_X['is_increasing'] = df_X['pow'] > df_X ['shifted_pow']
    df_X['slop'] = df_X['pow'] - df_X['shifted_pow']
    df_X.head()
    a = 0
    list_to_df = []
    for id_row, row in df_X.iterrows():
      x = row['is_increasing']*(1+a)
      a = x
      list_to_df.append(x)
    df_X['sum_of_elements_incresing'] = list_to_df
    
    
    element_list = df_X['sum_of_elements_incresing'].tolist()
    slop_list = df_X['slop'].tolist()
    pow_lst = df_X['pow'].tolist()
    
    
    slop_start_element = list(filter(lambda i:i > -30, slop_list))[0]
    
    ind = slop_list.index(slop_start_element)
    
    
    f_lst = f.tolist()
    real_dat = f_lst[ind:]
    sub_slop_list = slop_list[ind+10:]
    
    sub_slop = [sum(sub_slop_list[i:i+5]) for i in range(0, len(sub_slop_list ), 5)]
    start_recieve_slop = list(filter(lambda i: abs(i) > 15, sub_slop))[0]
    rec_ind = sub_slop.index(start_recieve_slop)
    rec_ind_mul = rec_ind * 5+10
    reciving_dat = real_dat[rec_ind_mul:]
    
    
    
    n=len(f)
    
    tmp = np.max(reciving_dat)
    minpo = np.min(reciving_dat)
    pos = np.where(reciving_dat == tmp)
    
    
    real_tmp = tmp/1000
    PmW = math.pow(10, real_tmp/10)
    Pt = 115
    c = 299792458
    d = c/(math.sqrt(PmW/Pt)*4*math.pi*426000000)
    
    time = 240  #global variable end time - start time !!!!!!!!!!!!!!!!! from RF_thread!!!!!
    dt = time/n
    t_max = dt*pos[0][0]
    yagi_sp = 0.36/0.216
    degree =t_max*yagi_sp
    final_location= [degree, d]
    print(final_location)

if __name__ == '__main__':
    main()

