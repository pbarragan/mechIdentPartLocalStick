# this is for the 2015/12/21 experiments
# no parameter variation. 50 trials for each model type
# these are simulations
# this is with simple action random
# no bias

files = ['data0Mon_Dec_21_11_01_00_2015.txt',
'data0Mon_Dec_21_11_01_18_2015.txt',
'data0Mon_Dec_21_11_01_35_2015.txt',
'data0Mon_Dec_21_11_01_53_2015.txt',
'data0Mon_Dec_21_11_02_10_2015.txt',
'data0Mon_Dec_21_11_02_27_2015.txt',
'data0Mon_Dec_21_11_02_45_2015.txt',
'data0Mon_Dec_21_11_03_02_2015.txt',
'data0Mon_Dec_21_11_03_19_2015.txt',
'data0Mon_Dec_21_11_03_37_2015.txt',
'data0Mon_Dec_21_11_03_54_2015.txt',
'data0Mon_Dec_21_11_04_13_2015.txt',
'data0Mon_Dec_21_11_04_31_2015.txt',
'data0Mon_Dec_21_11_04_49_2015.txt',
'data0Mon_Dec_21_11_05_06_2015.txt',
'data0Mon_Dec_21_11_05_23_2015.txt',
'data0Mon_Dec_21_11_05_41_2015.txt',
'data0Mon_Dec_21_11_05_57_2015.txt',
'data0Mon_Dec_21_11_06_15_2015.txt',
'data0Mon_Dec_21_11_06_32_2015.txt',
'data0Mon_Dec_21_11_06_50_2015.txt',
'data0Mon_Dec_21_11_07_07_2015.txt',
'data0Mon_Dec_21_11_07_25_2015.txt',
'data0Mon_Dec_21_11_07_42_2015.txt',
'data0Mon_Dec_21_11_08_01_2015.txt',
'data0Mon_Dec_21_11_08_18_2015.txt',
'data0Mon_Dec_21_11_08_35_2015.txt',
'data0Mon_Dec_21_11_08_52_2015.txt',
'data0Mon_Dec_21_11_09_10_2015.txt',
'data0Mon_Dec_21_11_09_27_2015.txt',
'data0Mon_Dec_21_11_09_44_2015.txt',
'data0Mon_Dec_21_11_10_01_2015.txt',
'data0Mon_Dec_21_11_10_18_2015.txt',
'data0Mon_Dec_21_11_10_35_2015.txt',
'data0Mon_Dec_21_11_10_54_2015.txt',
'data0Mon_Dec_21_11_11_11_2015.txt',
'data0Mon_Dec_21_11_11_30_2015.txt',
'data0Mon_Dec_21_11_11_47_2015.txt',
'data0Mon_Dec_21_11_12_05_2015.txt',
'data0Mon_Dec_21_11_12_22_2015.txt',
'data0Mon_Dec_21_11_12_41_2015.txt',
'data0Mon_Dec_21_11_12_58_2015.txt',
'data0Mon_Dec_21_11_13_16_2015.txt',
'data0Mon_Dec_21_11_13_35_2015.txt',
'data0Mon_Dec_21_11_13_52_2015.txt',
'data0Mon_Dec_21_11_14_12_2015.txt',
'data0Mon_Dec_21_11_14_30_2015.txt',
'data0Mon_Dec_21_11_14_48_2015.txt',
'data0Mon_Dec_21_11_15_06_2015.txt',
'data0Mon_Dec_21_11_15_24_2015.txt',
'data1Mon_Dec_21_11_15_41_2015.txt',
'data1Mon_Dec_21_11_15_59_2015.txt',
'data1Mon_Dec_21_11_16_15_2015.txt',
'data1Mon_Dec_21_11_16_33_2015.txt',
'data1Mon_Dec_21_11_16_50_2015.txt',
'data1Mon_Dec_21_11_17_08_2015.txt',
'data1Mon_Dec_21_11_17_25_2015.txt',
'data1Mon_Dec_21_11_17_43_2015.txt',
'data1Mon_Dec_21_11_18_00_2015.txt',
'data1Mon_Dec_21_11_18_18_2015.txt',
'data1Mon_Dec_21_11_18_35_2015.txt',
'data1Mon_Dec_21_11_18_53_2015.txt',
'data1Mon_Dec_21_11_19_10_2015.txt',
'data1Mon_Dec_21_11_19_28_2015.txt',
'data1Mon_Dec_21_11_19_45_2015.txt',
'data1Mon_Dec_21_11_20_03_2015.txt',
'data1Mon_Dec_21_11_20_20_2015.txt',
'data1Mon_Dec_21_11_20_38_2015.txt',
'data1Mon_Dec_21_11_20_55_2015.txt',
'data1Mon_Dec_21_11_21_13_2015.txt',
'data1Mon_Dec_21_11_21_30_2015.txt',
'data1Mon_Dec_21_11_21_47_2015.txt',
'data1Mon_Dec_21_11_22_05_2015.txt',
'data1Mon_Dec_21_11_22_22_2015.txt',
'data1Mon_Dec_21_11_22_39_2015.txt',
'data1Mon_Dec_21_11_22_57_2015.txt',
'data1Mon_Dec_21_11_23_14_2015.txt',
'data1Mon_Dec_21_11_23_31_2015.txt',
'data1Mon_Dec_21_11_23_49_2015.txt',
'data1Mon_Dec_21_11_24_06_2015.txt',
'data1Mon_Dec_21_11_24_23_2015.txt',
'data1Mon_Dec_21_11_24_41_2015.txt',
'data1Mon_Dec_21_11_24_58_2015.txt',
'data1Mon_Dec_21_11_25_15_2015.txt',
'data1Mon_Dec_21_11_25_33_2015.txt',
'data1Mon_Dec_21_11_25_50_2015.txt',
'data1Mon_Dec_21_11_26_07_2015.txt',
'data1Mon_Dec_21_11_26_25_2015.txt',
'data1Mon_Dec_21_11_26_43_2015.txt',
'data1Mon_Dec_21_11_27_00_2015.txt',
'data1Mon_Dec_21_11_27_17_2015.txt',
'data1Mon_Dec_21_11_27_34_2015.txt',
'data1Mon_Dec_21_11_27_52_2015.txt',
'data1Mon_Dec_21_11_28_10_2015.txt',
'data1Mon_Dec_21_11_28_28_2015.txt',
'data1Mon_Dec_21_11_28_45_2015.txt',
'data1Mon_Dec_21_11_29_02_2015.txt',
'data1Mon_Dec_21_11_29_20_2015.txt',
'data1Mon_Dec_21_11_29_37_2015.txt',
'data1Mon_Dec_21_11_29_55_2015.txt',
'data2Mon_Dec_21_11_30_12_2015.txt',
'data2Mon_Dec_21_11_30_30_2015.txt',
'data2Mon_Dec_21_11_30_47_2015.txt',
'data2Mon_Dec_21_11_31_04_2015.txt',
'data2Mon_Dec_21_11_31_21_2015.txt',
'data2Mon_Dec_21_11_31_38_2015.txt',
'data2Mon_Dec_21_11_31_56_2015.txt',
'data2Mon_Dec_21_11_32_13_2015.txt',
'data2Mon_Dec_21_11_32_30_2015.txt',
'data2Mon_Dec_21_11_32_47_2015.txt',
'data2Mon_Dec_21_11_33_05_2015.txt',
'data2Mon_Dec_21_11_33_22_2015.txt',
'data2Mon_Dec_21_11_33_39_2015.txt',
'data2Mon_Dec_21_11_33_56_2015.txt',
'data2Mon_Dec_21_11_34_14_2015.txt',
'data2Mon_Dec_21_11_34_31_2015.txt',
'data2Mon_Dec_21_11_34_48_2015.txt',
'data2Mon_Dec_21_11_35_05_2015.txt',
'data2Mon_Dec_21_11_35_22_2015.txt',
'data2Mon_Dec_21_11_35_39_2015.txt',
'data2Mon_Dec_21_11_35_57_2015.txt',
'data2Mon_Dec_21_11_36_14_2015.txt',
'data2Mon_Dec_21_11_36_31_2015.txt',
'data2Mon_Dec_21_11_36_49_2015.txt',
'data2Mon_Dec_21_11_37_06_2015.txt',
'data2Mon_Dec_21_11_37_24_2015.txt',
'data2Mon_Dec_21_11_37_41_2015.txt',
'data2Mon_Dec_21_11_37_58_2015.txt',
'data2Mon_Dec_21_11_38_16_2015.txt',
'data2Mon_Dec_21_11_38_33_2015.txt',
'data2Mon_Dec_21_11_38_50_2015.txt',
'data2Mon_Dec_21_11_39_08_2015.txt',
'data2Mon_Dec_21_11_39_25_2015.txt',
'data2Mon_Dec_21_11_39_42_2015.txt',
'data2Mon_Dec_21_11_40_00_2015.txt',
'data2Mon_Dec_21_11_40_17_2015.txt',
'data2Mon_Dec_21_11_40_35_2015.txt',
'data2Mon_Dec_21_11_40_53_2015.txt',
'data2Mon_Dec_21_11_41_10_2015.txt',
'data2Mon_Dec_21_11_41_27_2015.txt',
'data2Mon_Dec_21_11_41_44_2015.txt',
'data2Mon_Dec_21_11_42_02_2015.txt',
'data2Mon_Dec_21_11_42_20_2015.txt',
'data2Mon_Dec_21_11_42_37_2015.txt',
'data2Mon_Dec_21_11_42_55_2015.txt',
'data2Mon_Dec_21_11_43_12_2015.txt',
'data2Mon_Dec_21_11_43_29_2015.txt',
'data2Mon_Dec_21_11_43_47_2015.txt',
'data2Mon_Dec_21_11_44_04_2015.txt',
'data2Mon_Dec_21_11_44_21_2015.txt',
'data3Mon_Dec_21_11_44_38_2015.txt',
'data3Mon_Dec_21_11_44_55_2015.txt',
'data3Mon_Dec_21_11_45_13_2015.txt',
'data3Mon_Dec_21_11_45_31_2015.txt',
'data3Mon_Dec_21_11_45_48_2015.txt',
'data3Mon_Dec_21_11_46_06_2015.txt',
'data3Mon_Dec_21_11_46_23_2015.txt',
'data3Mon_Dec_21_11_46_41_2015.txt',
'data3Mon_Dec_21_11_46_59_2015.txt',
'data3Mon_Dec_21_11_47_16_2015.txt',
'data3Mon_Dec_21_11_47_34_2015.txt',
'data3Mon_Dec_21_11_47_52_2015.txt',
'data3Mon_Dec_21_11_48_09_2015.txt',
'data3Mon_Dec_21_11_48_27_2015.txt',
'data3Mon_Dec_21_11_48_45_2015.txt',
'data3Mon_Dec_21_11_49_02_2015.txt',
'data3Mon_Dec_21_11_49_19_2015.txt',
'data3Mon_Dec_21_11_49_37_2015.txt',
'data3Mon_Dec_21_11_49_54_2015.txt',
'data3Mon_Dec_21_11_50_12_2015.txt',
'data3Mon_Dec_21_11_50_29_2015.txt',
'data3Mon_Dec_21_11_50_47_2015.txt',
'data3Mon_Dec_21_11_51_04_2015.txt',
'data3Mon_Dec_21_11_51_22_2015.txt',
'data3Mon_Dec_21_11_51_40_2015.txt',
'data3Mon_Dec_21_11_51_57_2015.txt',
'data3Mon_Dec_21_11_52_14_2015.txt',
'data3Mon_Dec_21_11_52_32_2015.txt',
'data3Mon_Dec_21_11_52_50_2015.txt',
'data3Mon_Dec_21_11_53_07_2015.txt',
'data3Mon_Dec_21_11_53_25_2015.txt',
'data3Mon_Dec_21_11_53_42_2015.txt',
'data3Mon_Dec_21_11_54_01_2015.txt',
'data3Mon_Dec_21_11_54_18_2015.txt',
'data3Mon_Dec_21_11_54_35_2015.txt',
'data3Mon_Dec_21_11_54_53_2015.txt',
'data3Mon_Dec_21_11_55_11_2015.txt',
'data3Mon_Dec_21_11_55_28_2015.txt',
'data3Mon_Dec_21_11_55_46_2015.txt',
'data3Mon_Dec_21_11_56_04_2015.txt',
'data3Mon_Dec_21_11_56_21_2015.txt',
'data3Mon_Dec_21_11_56_38_2015.txt',
'data3Mon_Dec_21_11_56_56_2015.txt',
'data3Mon_Dec_21_11_57_12_2015.txt',
'data3Mon_Dec_21_11_57_30_2015.txt',
'data3Mon_Dec_21_11_57_48_2015.txt',
'data3Mon_Dec_21_11_58_06_2015.txt',
'data3Mon_Dec_21_11_58_24_2015.txt',
'data3Mon_Dec_21_11_58_42_2015.txt',
'data3Mon_Dec_21_11_58_58_2015.txt',
'data4Mon_Dec_21_11_59_16_2015.txt',
'data4Mon_Dec_21_11_59_33_2015.txt',
'data4Mon_Dec_21_11_59_51_2015.txt',
'data4Mon_Dec_21_12_00_09_2015.txt',
'data4Mon_Dec_21_12_00_27_2015.txt',
'data4Mon_Dec_21_12_00_44_2015.txt',
'data4Mon_Dec_21_12_01_02_2015.txt',
'data4Mon_Dec_21_12_01_19_2015.txt',
'data4Mon_Dec_21_12_01_37_2015.txt',
'data4Mon_Dec_21_12_01_56_2015.txt',
'data4Mon_Dec_21_12_02_13_2015.txt',
'data4Mon_Dec_21_12_02_31_2015.txt',
'data4Mon_Dec_21_12_02_49_2015.txt',
'data4Mon_Dec_21_12_03_06_2015.txt',
'data4Mon_Dec_21_12_03_24_2015.txt',
'data4Mon_Dec_21_12_03_42_2015.txt',
'data4Mon_Dec_21_12_04_00_2015.txt',
'data4Mon_Dec_21_12_04_17_2015.txt',
'data4Mon_Dec_21_12_04_34_2015.txt',
'data4Mon_Dec_21_12_04_52_2015.txt',
'data4Mon_Dec_21_12_05_09_2015.txt',
'data4Mon_Dec_21_12_05_28_2015.txt',
'data4Mon_Dec_21_12_05_45_2015.txt',
'data4Mon_Dec_21_12_06_03_2015.txt',
'data4Mon_Dec_21_12_06_21_2015.txt',
'data4Mon_Dec_21_12_06_39_2015.txt',
'data4Mon_Dec_21_12_06_56_2015.txt',
'data4Mon_Dec_21_12_07_15_2015.txt',
'data4Mon_Dec_21_12_07_33_2015.txt',
'data4Mon_Dec_21_12_07_50_2015.txt',
'data4Mon_Dec_21_12_08_08_2015.txt',
'data4Mon_Dec_21_12_08_25_2015.txt',
'data4Mon_Dec_21_12_08_42_2015.txt',
'data4Mon_Dec_21_12_09_00_2015.txt',
'data4Mon_Dec_21_12_09_18_2015.txt',
'data4Mon_Dec_21_12_09_35_2015.txt',
'data4Mon_Dec_21_12_09_53_2015.txt',
'data4Mon_Dec_21_12_10_10_2015.txt',
'data4Mon_Dec_21_12_10_28_2015.txt',
'data4Mon_Dec_21_12_10_46_2015.txt',
'data4Mon_Dec_21_12_11_03_2015.txt',
'data4Mon_Dec_21_12_11_21_2015.txt',
'data4Mon_Dec_21_12_11_39_2015.txt',
'data4Mon_Dec_21_12_11_57_2015.txt',
'data4Mon_Dec_21_12_12_14_2015.txt',
'data4Mon_Dec_21_12_12_32_2015.txt',
'data4Mon_Dec_21_12_12_50_2015.txt',
'data4Mon_Dec_21_12_13_08_2015.txt',
'data4Mon_Dec_21_12_13_25_2015.txt',
'data4Mon_Dec_21_12_13_43_2015.txt']
