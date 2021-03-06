# this is for the 2015/09/28 experiments
# no parameter variation. 50 trials for each model type
# these are simulations
# this is with simple action selection (round robin)
# sometimes depending on action validity, the action sequence is not identical
# 80% bias level

files = ['data0Mon_Sep_28_20_36_43_2015.txt',
'data0Mon_Sep_28_20_37_00_2015.txt',
'data0Mon_Sep_28_20_37_19_2015.txt',
'data0Mon_Sep_28_20_37_36_2015.txt',
'data0Mon_Sep_28_20_37_55_2015.txt',
'data0Mon_Sep_28_20_38_12_2015.txt',
'data0Mon_Sep_28_20_38_30_2015.txt',
'data0Mon_Sep_28_20_38_47_2015.txt',
'data0Mon_Sep_28_20_39_05_2015.txt',
'data0Mon_Sep_28_20_39_23_2015.txt',
'data0Mon_Sep_28_20_39_41_2015.txt',
'data0Mon_Sep_28_20_39_58_2015.txt',
'data0Mon_Sep_28_20_40_16_2015.txt',
'data0Mon_Sep_28_20_40_33_2015.txt',
'data0Mon_Sep_28_20_40_52_2015.txt',
'data0Mon_Sep_28_20_41_09_2015.txt',
'data0Mon_Sep_28_20_41_27_2015.txt',
'data0Mon_Sep_28_20_41_45_2015.txt',
'data0Mon_Sep_28_20_42_03_2015.txt',
'data0Mon_Sep_28_20_42_21_2015.txt',
'data0Mon_Sep_28_20_42_38_2015.txt',
'data0Mon_Sep_28_20_42_56_2015.txt',
'data0Mon_Sep_28_20_43_14_2015.txt',
'data0Mon_Sep_28_20_43_31_2015.txt',
'data0Mon_Sep_28_20_43_49_2015.txt',
'data0Mon_Sep_28_20_44_07_2015.txt',
'data0Mon_Sep_28_20_44_24_2015.txt',
'data0Mon_Sep_28_20_44_42_2015.txt',
'data0Mon_Sep_28_20_45_00_2015.txt',
'data0Mon_Sep_28_20_45_18_2015.txt',
'data0Mon_Sep_28_20_45_36_2015.txt',
'data0Mon_Sep_28_20_45_53_2015.txt',
'data0Mon_Sep_28_20_46_11_2015.txt',
'data0Mon_Sep_28_20_46_28_2015.txt',
'data0Mon_Sep_28_20_46_46_2015.txt',
'data0Mon_Sep_28_20_47_04_2015.txt',
'data0Mon_Sep_28_20_47_22_2015.txt',
'data0Mon_Sep_28_20_47_40_2015.txt',
'data0Mon_Sep_28_20_47_58_2015.txt',
'data0Mon_Sep_28_20_48_16_2015.txt',
'data0Mon_Sep_28_20_48_34_2015.txt',
'data0Mon_Sep_28_20_48_51_2015.txt',
'data0Mon_Sep_28_20_49_10_2015.txt',
'data0Mon_Sep_28_20_49_28_2015.txt',
'data0Mon_Sep_28_20_49_45_2015.txt',
'data0Mon_Sep_28_20_50_03_2015.txt',
'data0Mon_Sep_28_20_50_21_2015.txt',
'data0Mon_Sep_28_20_50_38_2015.txt',
'data0Mon_Sep_28_20_50_57_2015.txt',
'data0Mon_Sep_28_20_51_14_2015.txt',
'data1Mon_Sep_28_20_51_32_2015.txt',
'data1Mon_Sep_28_20_51_49_2015.txt',
'data1Mon_Sep_28_20_52_07_2015.txt',
'data1Mon_Sep_28_20_52_24_2015.txt',
'data1Mon_Sep_28_20_52_41_2015.txt',
'data1Mon_Sep_28_20_52_59_2015.txt',
'data1Mon_Sep_28_20_53_16_2015.txt',
'data1Mon_Sep_28_20_53_34_2015.txt',
'data1Mon_Sep_28_20_53_51_2015.txt',
'data1Mon_Sep_28_20_54_09_2015.txt',
'data1Mon_Sep_28_20_54_26_2015.txt',
'data1Mon_Sep_28_20_54_44_2015.txt',
'data1Mon_Sep_28_20_55_01_2015.txt',
'data1Mon_Sep_28_20_55_19_2015.txt',
'data1Mon_Sep_28_20_55_36_2015.txt',
'data1Mon_Sep_28_20_55_53_2015.txt',
'data1Mon_Sep_28_20_56_10_2015.txt',
'data1Mon_Sep_28_20_56_28_2015.txt',
'data1Mon_Sep_28_20_56_45_2015.txt',
'data1Mon_Sep_28_20_57_02_2015.txt',
'data1Mon_Sep_28_20_57_20_2015.txt',
'data1Mon_Sep_28_20_57_37_2015.txt',
'data1Mon_Sep_28_20_57_54_2015.txt',
'data1Mon_Sep_28_20_58_12_2015.txt',
'data1Mon_Sep_28_20_58_29_2015.txt',
'data1Mon_Sep_28_20_58_46_2015.txt',
'data1Mon_Sep_28_20_59_03_2015.txt',
'data1Mon_Sep_28_20_59_21_2015.txt',
'data1Mon_Sep_28_20_59_38_2015.txt',
'data1Mon_Sep_28_20_59_56_2015.txt',
'data1Mon_Sep_28_21_00_13_2015.txt',
'data1Mon_Sep_28_21_00_31_2015.txt',
'data1Mon_Sep_28_21_00_49_2015.txt',
'data1Mon_Sep_28_21_01_06_2015.txt',
'data1Mon_Sep_28_21_01_24_2015.txt',
'data1Mon_Sep_28_21_01_42_2015.txt',
'data1Mon_Sep_28_21_01_59_2015.txt',
'data1Mon_Sep_28_21_02_16_2015.txt',
'data1Mon_Sep_28_21_02_34_2015.txt',
'data1Mon_Sep_28_21_02_51_2015.txt',
'data1Mon_Sep_28_21_03_08_2015.txt',
'data1Mon_Sep_28_21_03_26_2015.txt',
'data1Mon_Sep_28_21_03_43_2015.txt',
'data1Mon_Sep_28_21_04_01_2015.txt',
'data1Mon_Sep_28_21_04_18_2015.txt',
'data1Mon_Sep_28_21_04_35_2015.txt',
'data1Mon_Sep_28_21_04_53_2015.txt',
'data1Mon_Sep_28_21_05_11_2015.txt',
'data1Mon_Sep_28_21_05_28_2015.txt',
'data1Mon_Sep_28_21_05_46_2015.txt',
'data2Mon_Sep_28_21_06_03_2015.txt',
'data2Mon_Sep_28_21_06_20_2015.txt',
'data2Mon_Sep_28_21_06_38_2015.txt',
'data2Mon_Sep_28_21_06_56_2015.txt',
'data2Mon_Sep_28_21_07_13_2015.txt',
'data2Mon_Sep_28_21_07_31_2015.txt',
'data2Mon_Sep_28_21_07_48_2015.txt',
'data2Mon_Sep_28_21_08_05_2015.txt',
'data2Mon_Sep_28_21_08_22_2015.txt',
'data2Mon_Sep_28_21_08_40_2015.txt',
'data2Mon_Sep_28_21_08_57_2015.txt',
'data2Mon_Sep_28_21_09_14_2015.txt',
'data2Mon_Sep_28_21_09_31_2015.txt',
'data2Mon_Sep_28_21_09_49_2015.txt',
'data2Mon_Sep_28_21_10_07_2015.txt',
'data2Mon_Sep_28_21_10_24_2015.txt',
'data2Mon_Sep_28_21_10_41_2015.txt',
'data2Mon_Sep_28_21_10_59_2015.txt',
'data2Mon_Sep_28_21_11_16_2015.txt',
'data2Mon_Sep_28_21_11_34_2015.txt',
'data2Mon_Sep_28_21_11_51_2015.txt',
'data2Mon_Sep_28_21_12_08_2015.txt',
'data2Mon_Sep_28_21_12_26_2015.txt',
'data2Mon_Sep_28_21_12_43_2015.txt',
'data2Mon_Sep_28_21_13_00_2015.txt',
'data2Mon_Sep_28_21_13_17_2015.txt',
'data2Mon_Sep_28_21_13_35_2015.txt',
'data2Mon_Sep_28_21_53_13_2015.txt',
'data2Mon_Sep_28_21_53_30_2015.txt',
'data2Mon_Sep_28_21_53_47_2015.txt',
'data2Mon_Sep_28_21_54_05_2015.txt',
'data2Mon_Sep_28_21_54_22_2015.txt',
'data2Mon_Sep_28_21_54_40_2015.txt',
'data2Mon_Sep_28_21_58_07_2015.txt',
'data2Mon_Sep_28_21_58_24_2015.txt',
'data2Mon_Sep_28_21_58_42_2015.txt',
'data2Mon_Sep_28_21_58_59_2015.txt',
'data2Mon_Sep_28_21_59_17_2015.txt',
'data2Mon_Sep_28_21_59_34_2015.txt',
'data2Mon_Sep_28_21_59_51_2015.txt',
'data2Mon_Sep_28_22_00_09_2015.txt',
'data2Mon_Sep_28_22_00_26_2015.txt',
'data2Mon_Sep_28_22_00_43_2015.txt',
'data2Mon_Sep_28_22_01_01_2015.txt',
'data2Mon_Sep_28_22_01_18_2015.txt',
'data2Mon_Sep_28_22_01_35_2015.txt',
'data2Mon_Sep_28_22_01_53_2015.txt',
'data2Mon_Sep_28_22_02_11_2015.txt',
'data2Mon_Sep_28_22_02_28_2015.txt',
'data2Mon_Sep_28_22_02_45_2015.txt',
'data3Mon_Sep_28_22_04_32_2015.txt',
'data3Mon_Sep_28_22_04_50_2015.txt',
'data3Mon_Sep_28_22_05_07_2015.txt',
'data3Mon_Sep_28_22_05_25_2015.txt',
'data3Mon_Sep_28_22_05_42_2015.txt',
'data3Mon_Sep_28_22_06_00_2015.txt',
'data3Mon_Sep_28_22_06_17_2015.txt',
'data3Mon_Sep_28_22_06_35_2015.txt',
'data3Mon_Sep_28_22_06_52_2015.txt',
'data3Mon_Sep_28_22_07_09_2015.txt',
'data3Mon_Sep_28_22_07_27_2015.txt',
'data3Mon_Sep_28_22_07_44_2015.txt',
'data3Mon_Sep_28_22_08_02_2015.txt',
'data3Mon_Sep_28_22_08_19_2015.txt',
'data3Mon_Sep_28_22_08_37_2015.txt',
'data3Mon_Sep_28_22_08_55_2015.txt',
'data3Mon_Sep_28_22_09_12_2015.txt',
'data3Mon_Sep_28_22_09_30_2015.txt',
'data3Mon_Sep_28_22_09_47_2015.txt',
'data3Mon_Sep_28_22_10_04_2015.txt',
'data3Mon_Sep_28_22_10_22_2015.txt',
'data3Mon_Sep_28_22_10_39_2015.txt',
'data3Mon_Sep_28_22_10_56_2015.txt',
'data3Mon_Sep_28_22_11_15_2015.txt',
'data3Mon_Sep_28_22_11_32_2015.txt',
'data3Mon_Sep_28_22_11_49_2015.txt',
'data3Mon_Sep_28_22_12_06_2015.txt',
'data3Mon_Sep_28_22_12_24_2015.txt',
'data3Mon_Sep_28_22_12_41_2015.txt',
'data3Mon_Sep_28_22_12_59_2015.txt',
'data3Mon_Sep_28_22_13_16_2015.txt',
'data3Mon_Sep_28_22_13_33_2015.txt',
'data3Mon_Sep_28_22_13_51_2015.txt',
'data3Mon_Sep_28_22_14_08_2015.txt',
'data3Mon_Sep_28_22_14_25_2015.txt',
'data3Mon_Sep_28_22_14_43_2015.txt',
'data3Mon_Sep_28_22_15_00_2015.txt',
'data3Mon_Sep_28_22_15_18_2015.txt',
'data3Mon_Sep_28_22_15_35_2015.txt',
'data3Mon_Sep_28_22_15_52_2015.txt',
'data3Mon_Sep_28_22_16_09_2015.txt',
'data3Mon_Sep_28_22_16_27_2015.txt',
'data3Mon_Sep_28_22_16_44_2015.txt',
'data3Mon_Sep_28_22_17_02_2015.txt',
'data3Mon_Sep_28_22_17_19_2015.txt',
'data3Mon_Sep_28_22_17_37_2015.txt',
'data3Mon_Sep_28_22_17_54_2015.txt',
'data3Mon_Sep_28_22_18_12_2015.txt',
'data3Mon_Sep_28_22_18_30_2015.txt',
'data3Mon_Sep_28_22_18_48_2015.txt',
'data4Mon_Sep_28_22_19_06_2015.txt',
'data4Mon_Sep_28_22_19_23_2015.txt',
'data4Mon_Sep_28_22_19_41_2015.txt',
'data4Mon_Sep_28_22_19_59_2015.txt',
'data4Mon_Sep_28_22_20_16_2015.txt',
'data4Mon_Sep_28_22_20_34_2015.txt',
'data4Mon_Sep_28_22_20_51_2015.txt',
'data4Mon_Sep_28_22_21_09_2015.txt',
'data4Mon_Sep_28_22_21_26_2015.txt',
'data4Mon_Sep_28_22_21_43_2015.txt',
'data4Mon_Sep_28_22_22_01_2015.txt',
'data4Mon_Sep_28_22_22_18_2015.txt',
'data4Mon_Sep_28_22_22_36_2015.txt',
'data4Mon_Sep_28_22_22_54_2015.txt',
'data4Mon_Sep_28_22_23_12_2015.txt',
'data4Mon_Sep_28_22_23_30_2015.txt',
'data4Mon_Sep_28_22_23_48_2015.txt',
'data4Mon_Sep_28_22_24_05_2015.txt',
'data4Mon_Sep_28_22_24_23_2015.txt',
'data4Mon_Sep_28_22_24_41_2015.txt',
'data4Mon_Sep_28_22_24_58_2015.txt',
'data4Mon_Sep_28_22_25_15_2015.txt',
'data4Mon_Sep_28_22_25_33_2015.txt',
'data4Mon_Sep_28_22_25_50_2015.txt',
'data4Mon_Sep_28_22_26_09_2015.txt',
'data4Mon_Sep_28_22_26_26_2015.txt',
'data4Mon_Sep_28_22_26_43_2015.txt',
'data4Mon_Sep_28_22_27_01_2015.txt',
'data4Mon_Sep_28_22_27_18_2015.txt',
'data4Mon_Sep_28_22_27_36_2015.txt',
'data4Mon_Sep_28_22_27_54_2015.txt',
'data4Mon_Sep_28_22_28_11_2015.txt',
'data4Mon_Sep_28_22_28_28_2015.txt',
'data4Mon_Sep_28_22_28_45_2015.txt',
'data4Mon_Sep_28_22_29_03_2015.txt',
'data4Mon_Sep_28_22_29_21_2015.txt',
'data4Mon_Sep_28_22_29_38_2015.txt',
'data4Mon_Sep_28_22_29_55_2015.txt',
'data4Mon_Sep_28_22_30_13_2015.txt',
'data4Mon_Sep_28_22_30_31_2015.txt',
'data4Mon_Sep_28_22_30_48_2015.txt',
'data4Mon_Sep_28_22_31_05_2015.txt',
'data4Mon_Sep_28_22_31_23_2015.txt',
'data4Mon_Sep_28_22_31_40_2015.txt',
'data4Mon_Sep_28_22_31_58_2015.txt',
'data4Mon_Sep_28_22_32_15_2015.txt',
'data4Mon_Sep_28_22_32_33_2015.txt',
'data4Mon_Sep_28_22_32_50_2015.txt',
'data4Mon_Sep_28_22_33_08_2015.txt',
'data4Mon_Sep_28_22_33_25_2015.txt']
