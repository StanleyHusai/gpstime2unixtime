"""
==================================
Fuzzy PID controller
==================================

Input variables
---------------

pro: probability score of NDT matching
Dpro: pro change rate (derivative of pro)

Output variable
---------------

kp: value of kp
kd: value of kd

We define 7 membership functions for 
both input variables and output variable. 
These are defined in scikit-fuzzy as follows

Date: 2/27/2019
By: Hu Sai
Email: 17095338g@connect.polyu.hk
"""
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

# Generate universe variables
x_pro = np.arange(0, 10, 0.1)
x_Dpro = np.arange(-5, 5, 0.1)
x_kp  = np.arange(0, 10, 0.1)
x_kd  = np.arange(0, 0.1, 0.001)

# Generate fuzzy membership functions
pro_ES = fuzz.trimf(x_pro, [0, 0, 1.667])
pro_VS = fuzz.trimf(x_pro, [0, 1.667, 3.333])
pro_S = fuzz.trimf(x_pro, [1.667, 3.333, 5])
pro_M = fuzz.trimf(x_pro, [3.333, 5, 6.667])
pro_L = fuzz.trimf(x_pro, [5, 6.667 8.333])
pro_VL = fuzz.trimf(x_pro, [6.667, 8.333, 10])
pro_EL = fuzz.trimf(x_pro, [8.333, 10, 10])

Dpro_NL = fuzz.trimf(x_Dpro, [-5, -5, -3.333])
Dpro_NM = fuzz.trimf(x_Dpro, [-5, -3.333, -1.667])
Dpro_NS = fuzz.trimf(x_Dpro, [-3.333, -1.667, 0])
Dpro_ZE = fuzz.trimf(x_Dpro, [-1.667, 0, 1.667])
Dpro_PS = fuzz.trimf(x_Dpro, [0, 1.667, 3.333])
Dpro_PM = fuzz.trimf(x_Dpro, [1.667, 3.333, 5])
Dpro_PL = fuzz.trimf(x_Dpro, [3.333, 5, 5])

kp_ES = fuzz.trimf(x_kp, [0, 0, 1.667])
kp_VS = fuzz.trimf(x_kp, [0, 1.667, 3.333])
kp_S = fuzz.trimf(x_kp, [1.667, 3.333, 5])
kp_M = fuzz.trimf(x_kp, [3.333, 5, 6.667])
kp_L = fuzz.trimf(x_kp, [5, 6.667 8.333])
kp_VL = fuzz.trimf(x_kp, [6.667, 8.333, 10])
kp_EL = fuzz.trimf(x_kp, [8.333, 10, 10])

kd_ES = fuzz.trimf(x_kd, [0, 0, 0.01666])
kd_VS = fuzz.trimf(x_kd, [0, 0.01666, 0.03333])
kd_S = fuzz.trimf(x_kd, [0.01666, 0.03333, 0.05])
kd_M = fuzz.trimf(x_kd, [0.0333, 0.05, 0.06663])
kd_L = fuzz.trimf(x_kd, [0.05, 0.06663, 0.08337])
kd_VL = fuzz.trimf(x_kd, [0.06663, 0.08337, 0.1])
kd_EL = fuzz.trimf(x_kd, [0.08337, 0.1, 0.1])

# Visualize these universes and membership functions
# fig, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=3, figsize=(8, 9))

# ax0.plot(x_pro, pro_ES, 'r', linewidth=1.5, label='ES')
# ax0.plot(x_pro, pro_VS, 'r', linewidth=1.5, label='VS')
# ax0.plot(x_pro, pro_S, 'r', linewidth=1.5, label='S')
# ax0.plot(x_pro, pro_M, 'r', linewidth=1.5, label='M')
# ax0.plot(x_pro, pro_L, 'r', linewidth=1.5, label='L')
# ax0.plot(x_pro, pro_VL, 'r', linewidth=1.5, label='VL')
# ax0.plot(x_pro, pro_EL, 'r', linewidth=1.5, label='EL')
# ax0.set_title('pro')
# ax0.legend()

# ax1.plot(x_Dpro, Dpro_NL, 'b', linewidth=1.5, label='NL')
# ax1.plot(x_Dpro, Dpro_NM, 'b', linewidth=1.5, label='NM')
# ax1.plot(x_Dpro, Dpro_NS, 'b', linewidth=1.5, label='NS')
# ax1.plot(x_Dpro, Dpro_ZE, 'b', linewidth=1.5, label='ZE')
# ax1.plot(x_Dpro, Dpro_PS, 'b', linewidth=1.5, label='PS')
# ax1.plot(x_Dpro, Dpro_PM, 'b', linewidth=1.5, label='PM')
# ax1.plot(x_Dpro, Dpro_PL, 'b', linewidth=1.5, label='PL')
# ax1.set_title('Dpro')
# ax1.legend()

# ax2.plot(x_kp, kp_ES, 'g', linewidth=1.5, label='ES')
# ax2.plot(x_kp, kp_VS, 'g', linewidth=1.5, label='VS')
# ax2.plot(x_kp, kp_S, 'g', linewidth=1.5, label='S')
# ax2.plot(x_kp, kp_M, 'g', linewidth=1.5, label='M')
# ax2.plot(x_kp, kp_L, 'g', linewidth=1.5, label='L')
# ax2.plot(x_kp, kp_VL, 'g', linewidth=1.5, label='VL')
# ax2.plot(x_kp, kp_EL, 'g', linewidth=1.5, label='EL')
# ax2.set_title('kp')
# ax2.legend()

# ax3.plot(x_kd, kd_ES, 'o', linewidth=1.5, label='ES')
# ax3.plot(x_kd, kd_VS, 'o', linewidth=1.5, label='VS')
# ax3.plot(x_kd, kd_S, 'o', linewidth=1.5, label='S')
# ax3.plot(x_kd, kd_M, 'o', linewidth=1.5, label='M')
# ax3.plot(x_kd, kd_L, 'o', linewidth=1.5, label='L')
# ax3.plot(x_kd, kd_VL, 'o', linewidth=1.5, label='VL')
# ax3.plot(x_kd, kd_EL, 'o', linewidth=1.5, label='EL')
# ax3.set_title('kd')
# ax3.legend()

# Turn off top/right axes
# for ax in (ax0, ax1, ax2, ax3):
#     ax.spines['top'].set_visible(False)
#     ax.spines['right'].set_visible(False)
#     ax.get_xaxis().tick_bottom()
#     ax.get_yaxis().tick_left()

# plt.tight_layout()
# Determine the degree
# We need the activation of our fuzzy membership functions at these values.
# The exact values 6.5 and 9.8 do not exist on our universes...
# This is what fuzz.interp_membership exists for!
pro_level_ES = fuzz.interp_membership(x_pro, pro_ES, ndt_reliability)
pro_level_VS = fuzz.interp_membership(x_pro, pro_VS, ndt_reliability)
pro_level_S = fuzz.interp_membership(x_pro, pro_S, ndt_reliability)
pro_level_M = fuzz.interp_membership(x_pro, pro_M, ndt_reliability)
pro_level_L = fuzz.interp_membership(x_pro, pro_L, ndt_reliability)
pro_level_VL = fuzz.interp_membership(x_pro, pro_VL, ndt_reliability)
pro_level_EL = fuzz.interp_membership(x_pro, pro_EL, ndt_reliability)

Dpro_level_NL = fuzz.interp_membership(x_Dpro, Dpro_NL, D_ndt_reliability)
Dpro_level_NM = fuzz.interp_membership(x_Dpro, Dpro_NM, D_ndt_reliability)
Dpro_level_NS = fuzz.interp_membership(x_Dpro, Dpro_NS, D_ndt_reliability)
Dpro_level_ZE = fuzz.interp_membership(x_Dpro, Dpro_ZE, D_ndt_reliability)
Dpro_level_PS = fuzz.interp_membership(x_Dpro, Dpro_PS, D_ndt_reliability)
Dpro_level_PM = fuzz.interp_membership(x_Dpro, Dpro_PM, D_ndt_reliability)
Dpro_level_PL = fuzz.interp_membership(x_Dpro, Dpro_PL, D_ndt_reliability)
# Implication
# Now we take our rules and apply them. Rule 1 is: if(pro is ES) and (D-pro is NL) then (kp is ES)(kd is ES).
# The AND operator means we take the min of these two.
# Then we apply this by clipping the top off the corresponding output
# membership function with np.fmin
active_rule1 = np.fmin(pro_level_ES, Dpro_level_NL)
kp_activation_rule1 = np.fmin(active_rule1, kp_ES)
kd_activation_rule1 = np.fmin(active_rule1, kd_ES)

active_rule2 = np.fmin(pro_level_ES, Dpro_level_NM)
kp_activation_rule2 = np.fmin(active_rule2, kp_ES)
kd_activation_rule2 = np.fmin(active_rule2, kd_ES)

active_rule3 = np.fmin(pro_level_ES, Dpro_level_NS)
kp_activation_rule3 = np.fmin(active_rule3, kp_ES)
kd_activation_rule3 = np.fmin(active_rule3, kd_ES)

active_rule4 = np.fmin(pro_level_ES, Dpro_level_ZE)
kp_activation_rule4 = np.fmin(active_rule4, kp_ES)
kd_activation_rule4 = np.fmin(active_rule4, kd_EL)

active_rule5 = np.fmin(pro_level_ES, Dpro_level_PS)
kp_activation_rule5 = np.fmin(active_rule5, kp_ES)
kd_activation_rule5 = np.fmin(active_rule5, kd_ES)

active_rule6 = np.fmin(pro_level_ES, Dpro_level_PM)
kp_activation_rule6 = np.fmin(active_rule6, kp_ES)
kd_activation_rule6 = np.fmin(active_rule6, kd_ES)

active_rule7 = np.fmin(pro_level_ES, Dpro_level_PL)
kp_activation_rule7 = np.fmin(active_rule7, kp_ES)
kd_activation_rule7 = np.fmin(active_rule7, kd_ES)

active_rule8 = np.fmin(pro_level_VS, Dpro_level_NL)
kp_activation_rule8 = np.fmin(active_rule8, kp_ES)
kd_activation_rule8 = np.fmin(active_rule8, kd_ES)

active_rule9 = np.fmin(pro_level_VS, Dpro_level_NM)
kp_activation_rule9 = np.fmin(active_rule9, kp_ES)
kd_activation_rule9 = np.fmin(active_rule9, kd_ES)

active_rule10 = np.fmin(pro_level_VS, Dpro_level_NS)
kp_activation_rule10 = np.fmin(active_rule10, kp_ES)
kd_activation_rule10 = np.fmin(active_rule10, kd_ES)

active_rule11 = np.fmin(pro_level_VS, Dpro_level_ZE)
kp_activation_rule11 = np.fmin(active_rule11, kp_ES)
kd_activation_rule11 = np.fmin(active_rule11, kd_EL)

active_rule12 = np.fmin(pro_level_VS, Dpro_level_PS)
kp_activation_rule12 = np.fmin(active_rule12, kp_ES)
kd_activation_rule12 = np.fmin(active_rule12, kd_ES)

active_rule13 = np.fmin(pro_level_VS, Dpro_level_PM)
kp_activation_rule13 = np.fmin(active_rule13, kp_ES)
kd_activation_rule13 = np.fmin(active_rule13, kd_ES)

active_rule14 = np.fmin(pro_level_VS, Dpro_level_PL)
kp_activation_rule14 = np.fmin(active_rule14, kp_ES)
kd_activation_rule14 = np.fmin(active_rule14, kd_ES)

active_rule15 = np.fmin(pro_level_S, Dpro_level_NL)
kp_activation_rule15 = np.fmin(active_rule15, kp_ES)
kd_activation_rule15 = np.fmin(active_rule15, kd_ES)

active_rule16 = np.fmin(pro_level_S, Dpro_level_NM)
kp_activation_rule16 = np.fmin(active_rule16, kp_ES)
kd_activation_rule16 = np.fmin(active_rule16, kd_ES)

active_rule17 = np.fmin(pro_level_S, Dpro_level_NS)
kp_activation_rule17 = np.fmin(active_rule17, kp_ES)
kd_activation_rule17 = np.fmin(active_rule17, kd_ES)

active_rule18 = np.fmin(pro_level_S, Dpro_level_ZE)
kp_activation_rule18 = np.fmin(active_rule18, kp_ES)
kd_activation_rule18 = np.fmin(active_rule18, kd_EL)

active_rule19 = np.fmin(pro_level_S, Dpro_level_PS)
kp_activation_rule19 = np.fmin(active_rule19, kp_ES)
kd_activation_rule19 = np.fmin(active_rule19, kd_ES)

active_rule20 = np.fmin(pro_level_S, Dpro_level_PM)
kp_activation_rule20 = np.fmin(active_rule20, kp_ES)
kd_activation_rule20 = np.fmin(active_rule20, kd_ES)

active_rule21 = np.fmin(pro_level_S, Dpro_level_PL)
kp_activation_rule21 = np.fmin(active_rule21, kp_ES)
kd_activation_rule21 = np.fmin(active_rule21, kd_ES)

active_rule22 = np.fmin(pro_level_M, Dpro_level_NL)
kp_activation_rule22 = np.fmin(active_rule22, kp_ES)
kd_activation_rule22 = np.fmin(active_rule22, kd_ES)

active_rule23 = np.fmin(pro_level_M, Dpro_level_NM)
kp_activation_rule23 = np.fmin(active_rule23, kp_ES)
kd_activation_rule23 = np.fmin(active_rule23, kd_ES)

active_rule24 = np.fmin(pro_level_M, Dpro_level_NS)
kp_activation_rule24 = np.fmin(active_rule24, kp_ES)
kd_activation_rule24 = np.fmin(active_rule24, kd_ES)

active_rule25 = np.fmin(pro_level_M, Dpro_level_ZE)
kp_activation_rule25 = np.fmin(active_rule25, kp_ES)
kd_activation_rule25 = np.fmin(active_rule25, kd_EL)

active_rule26 = np.fmin(pro_level_M, Dpro_level_PS)
kp_activation_rule26 = np.fmin(active_rule26, kp_ES)
kd_activation_rule26 = np.fmin(active_rule26, kd_ES)

active_rule27 = np.fmin(pro_level_M, Dpro_level_PM)
kp_activation_rule27 = np.fmin(active_rule27, kp_ES)
kd_activation_rule27 = np.fmin(active_rule27, kd_ES)

active_rule28 = np.fmin(pro_level_M, Dpro_level_PL)
kp_activation_rule28 = np.fmin(active_rule28, kp_ES)
kd_activation_rule28 = np.fmin(active_rule28, kd_ES)

active_rule29 = np.fmin(pro_level_L, Dpro_level_NL)
kp_activation_rule29 = np.fmin(active_rule29, kp_ES)
kd_activation_rule29 = np.fmin(active_rule29, kd_ES)

active_rule30 = np.fmin(pro_level_L, Dpro_level_NM)
kp_activation_rule30 = np.fmin(active_rule30, kp_ES)
kd_activation_rule30 = np.fmin(active_rule30, kd_ES)

active_rule31 = np.fmin(pro_level_L, Dpro_level_NS)
kp_activation_rule31 = np.fmin(active_rule31, kp_ES)
kd_activation_rule31 = np.fmin(active_rule31, kd_ES)

active_rule32 = np.fmin(pro_level_L, Dpro_level_ZE)
kp_activation_rule32 = np.fmin(active_rule32, kp_ES)
kd_activation_rule32 = np.fmin(active_rule32, kd_EL)

active_rule33 = np.fmin(pro_level_L, Dpro_level_PS)
kp_activation_rule33 = np.fmin(active_rule33, kp_ES)
kd_activation_rule33 = np.fmin(active_rule33, kd_ES)

active_rule34 = np.fmin(pro_level_L, Dpro_level_PM)
kp_activation_rule34 = np.fmin(active_rule34, kp_ES)
kd_activation_rule34 = np.fmin(active_rule34, kd_ES)

active_rule35 = np.fmin(pro_level_L, Dpro_level_PL)
kp_activation_rule35 = np.fmin(active_rule35, kp_ES)
kd_activation_rule35 = np.fmin(active_rule35, kd_ES)

active_rule36 = np.fmin(pro_level_VL, Dpro_level_NL)
kp_activation_rule36 = np.fmin(active_rule36, kp_EL)
kd_activation_rule36 = np.fmin(active_rule36, kd_ES)

active_rule37 = np.fmin(pro_level_VL, Dpro_level_NM)
kp_activation_rule37 = np.fmin(active_rule37, kp_EL)
kd_activation_rule37 = np.fmin(active_rule37, kd_ES)

active_rule38 = np.fmin(pro_level_VL, Dpro_level_NS)
kp_activation_rule38 = np.fmin(active_rule38, kp_EL)
kd_activation_rule38 = np.fmin(active_rule38, kd_ES)

active_rule39 = np.fmin(pro_level_VL, Dpro_level_ZE)
kp_activation_rule39 = np.fmin(active_rule39, kp_EL)
kd_activation_rule39 = np.fmin(active_rule39, kd_EL)

active_rule40 = np.fmin(pro_level_VL, Dpro_level_PS)
kp_activation_rule40 = np.fmin(active_rule40, kp_EL)
kd_activation_rule40 = np.fmin(active_rule40, kd_ES)

active_rule41 = np.fmin(pro_level_VL, Dpro_level_PM)
kp_activation_rule41 = np.fmin(active_rule41, kp_EL)
kd_activation_rule41 = np.fmin(active_rule41, kd_ES)

active_rule42 = np.fmin(pro_level_VL, Dpro_level_PL)
kp_activation_rule42 = np.fmin(active_rule42, kp_EL)
kd_activation_rule42 = np.fmin(active_rule42, kd_ES)

active_rule43 = np.fmin(pro_level_EL, Dpro_level_NL)
kp_activation_rule43 = np.fmin(active_rule43, kp_EL)
kd_activation_rule43 = np.fmin(active_rule43, kd_ES)

active_rule44 = np.fmin(pro_level_EL, Dpro_level_NM)
kp_activation_rule44 = np.fmin(active_rule44, kp_EL)
kd_activation_rule44 = np.fmin(active_rule44, kd_ES)

active_rule45 = np.fmin(pro_level_EL, Dpro_level_NS)
kp_activation_rule45 = np.fmin(active_rule45, kp_EL)
kd_activation_rule45 = np.fmin(active_rule45, kd_ES)

active_rule46 = np.fmin(pro_level_EL, Dpro_level_ZE)
kp_activation_rule46 = np.fmin(active_rule46, kp_EL)
kd_activation_rule46 = np.fmin(active_rule46, kd_EL)

active_rule47 = np.fmin(pro_level_EL, Dpro_level_PS)
kp_activation_rule47 = np.fmin(active_rule47, kp_EL)
kd_activation_rule47 = np.fmin(active_rule47, kd_ES)

active_rule48 = np.fmin(pro_level_EL, Dpro_level_PM)
kp_activation_rule48 = np.fmin(active_rule48, kp_EL)
kd_activation_rule48 = np.fmin(active_rule48, kd_ES)

active_rule49 = np.fmin(pro_level_EL, Dpro_level_PL)
kp_activation_rule49 = np.fmin(active_rule49, kp_EL)
kd_activation_rule49 = np.fmin(active_rule49, kd_ES)


# Visualize this
#fig, (ax0 = plt.subplots(figsize=(8, 3))

#ax0.fill_between(x_kp, kp0, kp_activation_lo, facecolor='b', alpha=0.7)
#ax0.plot(x_kp, kp_lo, 'b', linewidth=0.5, linestyle='--', )
#ax0.fill_between(x_kp, kp0, kp_activation_md, facecolor='g', alpha=0.7)
#ax0.plot(x_kp, kp_md, 'g', linewidth=0.5, linestyle='--')
#ax0.fill_between(x_kp, kp0, kp_activation_hi, facecolor='r', alpha=0.7)
#ax0.plot(x_kp, kp_hi, 'r', linewidth=0.5, linestyle='--')
#ax0.set_title('Output membership activity')

# Turn off top/right axes
#for ax in (ax0,):
    #ax.spines['top'].set_visible(False)
    #ax.spines['right'].set_visible(False)
    #ax.get_xaxis().tick_bottom()
    #ax.get_yaxis().tick_left()

#plt.tight_layout()

# Aggregate all output membership functions together
aggregated_kp1 = np.fmax(kp_activation_rule1, kp_activation_rule2)
aggregated_kp2 = np.fmax(aggregated_kp1, kp_activation_rule3)
aggregated_kp3 = np.fmax(aggregated_kp2, kp_activation_rule4)
aggregated_kp4 = np.fmax(aggregated_kp3, kp_activation_rule5)
aggregated_kp5 = np.fmax(aggregated_kp4, kp_activation_rule6)
aggregated_kp6 = np.fmax(aggregated_kp5, kp_activation_rule7)
aggregated_kp7 = np.fmax(aggregated_kp6, kp_activation_rule7)
aggregated_kp8 = np.fmax(aggregated_kp7, kp_activation_rule8)
aggregated_kp9 = np.fmax(aggregated_kp8, kp_activation_rule9)
aggregated_kp10 = np.fmax(aggregated_kp9, kp_activation_rule10)
aggregated_kp11 = np.fmax(aggregated_kp10, kp_activation_rule11)
aggregated_kp12 = np.fmax(aggregated_kp11, kp_activation_rule12)
aggregated_kp13 = np.fmax(aggregated_kp12, kp_activation_rule13)
aggregated_kp14 = np.fmax(aggregated_kp13, kp_activation_rule14)
aggregated_kp15 = np.fmax(aggregated_kp14, kp_activation_rule15)
aggregated_kp16 = np.fmax(aggregated_kp15, kp_activation_rule16)
aggregated_kp17 = np.fmax(aggregated_kp16, kp_activation_rule17)
aggregated_kp18 = np.fmax(aggregated_kp17, kp_activation_rule18)
aggregated_kp19 = np.fmax(aggregated_kp18, kp_activation_rule19)
aggregated_kp20 = np.fmax(aggregated_kp19, kp_activation_rule20)
aggregated_kp21 = np.fmax(aggregated_kp20, kp_activation_rule21)
aggregated_kp22 = np.fmax(aggregated_kp21, kp_activation_rule22)
aggregated_kp23 = np.fmax(aggregated_kp22, kp_activation_rule23)
aggregated_kp24 = np.fmax(aggregated_kp23, kp_activation_rule24)
aggregated_kp25 = np.fmax(aggregated_kp24, kp_activation_rule25)
aggregated_kp26 = np.fmax(aggregated_kp25, kp_activation_rule26)
aggregated_kp27 = np.fmax(aggregated_kp26, kp_activation_rule27)
aggregated_kp28 = np.fmax(aggregated_kp27, kp_activation_rule28)
aggregated_kp29 = np.fmax(aggregated_kp28, kp_activation_rule29)
aggregated_kp30 = np.fmax(aggregated_kp29, kp_activation_rule30)
aggregated_kp31 = np.fmax(aggregated_kp30, kp_activation_rule31)
aggregated_kp32 = np.fmax(aggregated_kp31, kp_activation_rule32)
aggregated_kp33 = np.fmax(aggregated_kp32, kp_activation_rule33)
aggregated_kp34 = np.fmax(aggregated_kp33, kp_activation_rule34)
aggregated_kp35 = np.fmax(aggregated_kp34, kp_activation_rule35)
aggregated_kp36 = np.fmax(aggregated_kp35, kp_activation_rule36)
aggregated_kp37 = np.fmax(aggregated_kp36, kp_activation_rule37)
aggregated_kp38 = np.fmax(aggregated_kp37, kp_activation_rule38)
aggregated_kp39 = np.fmax(aggregated_kp38, kp_activation_rule39)
aggregated_kp40 = np.fmax(aggregated_kp39, kp_activation_rule40)
aggregated_kp41 = np.fmax(aggregated_kp40, kp_activation_rule41)
aggregated_kp42 = np.fmax(aggregated_kp41, kp_activation_rule42)
aggregated_kp43 = np.fmax(aggregated_kp42, kp_activation_rule43)
aggregated_kp44 = np.fmax(aggregated_kp43, kp_activation_rule44)
aggregated_kp45 = np.fmax(aggregated_kp44, kp_activation_rule45)
aggregated_kp46 = np.fmax(aggregated_kp45, kp_activation_rule46)
aggregated_kp47 = np.fmax(aggregated_kp46, kp_activation_rule47)
aggregated_kp48 = np.fmax(aggregated_kp47, kp_activation_rule48)
aggregated_kp49 = np.fmax(aggregated_kp48, kp_activation_rule49)
aggregated_kp = aggregated_kp49

aggregated_kd1 = np.fmax(kd_activation_rule1, kd_activation_rule2)
aggregated_kd2 = np.fmax(aggregated_kd1, kd_activation_rule3)
aggregated_kd3 = np.fmax(aggregated_kd2, kd_activation_rule4)
aggregated_kd4 = np.fmax(aggregated_kd3, kd_activation_rule5)
aggregated_kd5 = np.fmax(aggregated_kd4, kd_activation_rule6)
aggregated_kd6 = np.fmax(aggregated_kd5, kd_activation_rule7)
aggregated_kd7 = np.fmax(aggregated_kd6, kd_activation_rule7)
aggregated_kd8 = np.fmax(aggregated_kd7, kd_activation_rule8)
aggregated_kd9 = np.fmax(aggregated_kd8, kd_activation_rule9)
aggregated_kd10 = np.fmax(aggregated_kd9, kd_activation_rule10)
aggregated_kd11 = np.fmax(aggregated_kd10, kd_activation_rule11)
aggregated_kd12 = np.fmax(aggregated_kd11, kd_activation_rule12)
aggregated_kd13 = np.fmax(aggregated_kd12, kd_activation_rule13)
aggregated_kd14 = np.fmax(aggregated_kd13, kd_activation_rule14)
aggregated_kd15 = np.fmax(aggregated_kd14, kd_activation_rule15)
aggregated_kd16 = np.fmax(aggregated_kd15, kd_activation_rule16)
aggregated_kd17 = np.fmax(aggregated_kd16, kd_activation_rule17)
aggregated_kd18 = np.fmax(aggregated_kd17, kd_activation_rule18)
aggregated_kd19 = np.fmax(aggregated_kd18, kd_activation_rule19)
aggregated_kd20 = np.fmax(aggregated_kd19, kd_activation_rule20)
aggregated_kd21 = np.fmax(aggregated_kd20, kd_activation_rule21)
aggregated_kd22 = np.fmax(aggregated_kd21, kd_activation_rule22)
aggregated_kd23 = np.fmax(aggregated_kd22, kd_activation_rule23)
aggregated_kd24 = np.fmax(aggregated_kd23, kd_activation_rule24)
aggregated_kd25 = np.fmax(aggregated_kd24, kd_activation_rule25)
aggregated_kd26 = np.fmax(aggregated_kd25, kd_activation_rule26)
aggregated_kd27 = np.fmax(aggregated_kd26, kd_activation_rule27)
aggregated_kd28 = np.fmax(aggregated_kd27, kd_activation_rule28)
aggregated_kd29 = np.fmax(aggregated_kd28, kd_activation_rule29)
aggregated_kd30 = np.fmax(aggregated_kd29, kd_activation_rule30)
aggregated_kd31 = np.fmax(aggregated_kd30, kd_activation_rule31)
aggregated_kd32 = np.fmax(aggregated_kd31, kd_activation_rule32)
aggregated_kd33 = np.fmax(aggregated_kd32, kd_activation_rule33)
aggregated_kd34 = np.fmax(aggregated_kd33, kd_activation_rule34)
aggregated_kd35 = np.fmax(aggregated_kd34, kd_activation_rule35)
aggregated_kd36 = np.fmax(aggregated_kd35, kd_activation_rule36)
aggregated_kd37 = np.fmax(aggregated_kd36, kd_activation_rule37)
aggregated_kd38 = np.fmax(aggregated_kd37, kd_activation_rule38)
aggregated_kd39 = np.fmax(aggregated_kd38, kd_activation_rule39)
aggregated_kd40 = np.fmax(aggregated_kd39, kd_activation_rule40)
aggregated_kd41 = np.fmax(aggregated_kd40, kd_activation_rule41)
aggregated_kd42 = np.fmax(aggregated_kd41, kd_activation_rule42)
aggregated_kd43 = np.fmax(aggregated_kd42, kd_activation_rule43)
aggregated_kd44 = np.fmax(aggregated_kd43, kd_activation_rule44)
aggregated_kd45 = np.fmax(aggregated_kd44, kd_activation_rule45)
aggregated_kd46 = np.fmax(aggregated_kd45, kd_activation_rule46)
aggregated_kd47 = np.fmax(aggregated_kd46, kd_activation_rule47)
aggregated_kd48 = np.fmax(aggregated_kd47, kd_activation_rule48)
aggregated_kd49 = np.fmax(aggregated_kd48, kd_activation_rule49)
aggregated_kd = aggregated_kd49

# Calculate defuzzified result
kp = fuzz.defuzz(x_kp, aggregated_kp, 'centroid')
kd = fuzz.defuzz(x_kd, aggregated_kd, 'centroid')

#kp_activation = fuzz.interp_membership(x_kp, aggregated, kp)  # for plot

# Visualize this
#fig, ax0 = plt.subplots(figsize=(8, 3))

#ax0.plot(x_kp, kp_lo, 'b', linewidth=0.5, linestyle='--', )
#ax0.plot(x_kp, kp_md, 'g', linewidth=0.5, linestyle='--')
#ax0.plot(x_kp, kp_hi, 'r', linewidth=0.5, linestyle='--')
#ax0.fill_between(x_kp, kp0, aggregated, facecolor='Orange', alpha=0.7)
#ax0.plot([kp, kp], [0, kp_activation], 'k', linewidth=1.5, alpha=0.9)
#ax0.set_title('Aggregated membership and result (line)')

# Turn off top/right axes
#for ax in (ax0,):
    #ax.spines['top'].set_visible(False)
    #ax.spines['right'].set_visible(False)
    #ax.get_xaxis().tick_bottom()
    #ax.get_yaxis().tick_left()

#plt.tight_layout()

"""
.. image:: PLOT2RST.current_figure

Final thoughts
--------------

The power of fuzzy systems is allowing complicated, intuitive behavior based
on a sparse system of rules with minimal overhead. Note our membership
function universes were coarse, only defined at the integers, but
``fuzz.interp_membership`` allowed the effective resolution to increase on
demand. This system can respond to arbitrarily small changes in inputs,
and the processing burden is minimal.

"""