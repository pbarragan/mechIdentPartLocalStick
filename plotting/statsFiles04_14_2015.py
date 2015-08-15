# this is for the 2015/04/14 experiments
# no parameter variation. 50 trials for each model type.
# no bias, random

files=['data0Tue_Apr_14_10_55_59_2015.txt',
'data0Tue_Apr_14_10_56_18_2015.txt',
'data0Tue_Apr_14_10_56_35_2015.txt',
'data0Tue_Apr_14_10_56_53_2015.txt',
'data0Tue_Apr_14_10_57_12_2015.txt',
'data0Tue_Apr_14_10_57_30_2015.txt',
'data0Tue_Apr_14_10_57_48_2015.txt',
'data0Tue_Apr_14_10_58_06_2015.txt',
'data0Tue_Apr_14_10_58_24_2015.txt',
'data0Tue_Apr_14_10_58_41_2015.txt',
'data0Tue_Apr_14_10_58_59_2015.txt',
'data0Tue_Apr_14_10_59_19_2015.txt',
'data0Tue_Apr_14_10_59_36_2015.txt',
'data0Tue_Apr_14_10_59_54_2015.txt',
'data0Tue_Apr_14_11_00_12_2015.txt',
'data0Tue_Apr_14_11_00_30_2015.txt',
'data0Tue_Apr_14_11_00_47_2015.txt',
'data0Tue_Apr_14_11_01_05_2015.txt',
'data0Tue_Apr_14_11_01_23_2015.txt',
'data0Tue_Apr_14_11_01_41_2015.txt',
'data0Tue_Apr_14_11_01_58_2015.txt',
'data0Tue_Apr_14_11_02_17_2015.txt',
'data0Tue_Apr_14_11_02_35_2015.txt',
'data0Tue_Apr_14_11_02_53_2015.txt',
'data0Tue_Apr_14_11_03_11_2015.txt',
'data0Tue_Apr_14_11_03_28_2015.txt',
'data0Tue_Apr_14_11_03_46_2015.txt',
'data0Tue_Apr_14_11_04_04_2015.txt',
'data0Tue_Apr_14_11_04_21_2015.txt',
'data0Tue_Apr_14_11_04_39_2015.txt',
'data0Tue_Apr_14_11_04_57_2015.txt',
'data0Tue_Apr_14_11_05_15_2015.txt',
'data0Tue_Apr_14_11_05_33_2015.txt',
'data0Tue_Apr_14_11_05_51_2015.txt',
'data0Tue_Apr_14_11_06_09_2015.txt',
'data0Tue_Apr_14_11_06_27_2015.txt',
'data0Tue_Apr_14_11_06_45_2015.txt',
'data0Tue_Apr_14_11_07_02_2015.txt',
'data0Tue_Apr_14_11_07_20_2015.txt',
'data0Tue_Apr_14_11_07_38_2015.txt',
'data0Tue_Apr_14_11_07_57_2015.txt',
'data0Tue_Apr_14_11_08_15_2015.txt',
'data0Tue_Apr_14_11_08_32_2015.txt',
'data0Tue_Apr_14_11_08_49_2015.txt',
'data0Tue_Apr_14_11_09_07_2015.txt',
'data0Tue_Apr_14_11_09_26_2015.txt',
'data0Tue_Apr_14_11_09_44_2015.txt',
'data0Tue_Apr_14_11_10_02_2015.txt',
'data0Tue_Apr_14_11_10_20_2015.txt',
'data0Tue_Apr_14_11_10_38_2015.txt',
'data1Tue_Apr_14_11_10_55_2015.txt',
'data1Tue_Apr_14_11_11_13_2015.txt',
'data1Tue_Apr_14_11_11_31_2015.txt',
'data1Tue_Apr_14_11_11_48_2015.txt',
'data1Tue_Apr_14_11_12_06_2015.txt',
'data1Tue_Apr_14_11_12_24_2015.txt',
'data1Tue_Apr_14_11_12_41_2015.txt',
'data1Tue_Apr_14_11_12_59_2015.txt',
'data1Tue_Apr_14_11_13_16_2015.txt',
'data1Tue_Apr_14_11_13_34_2015.txt',
'data1Tue_Apr_14_11_13_52_2015.txt',
'data1Tue_Apr_14_11_14_09_2015.txt',
'data1Tue_Apr_14_11_14_27_2015.txt',
'data1Tue_Apr_14_11_14_45_2015.txt',
'data1Tue_Apr_14_11_15_02_2015.txt',
'data1Tue_Apr_14_11_15_20_2015.txt',
'data1Tue_Apr_14_11_15_37_2015.txt',
'data1Tue_Apr_14_11_15_55_2015.txt',
'data1Tue_Apr_14_11_16_12_2015.txt',
'data1Tue_Apr_14_11_16_29_2015.txt',
'data1Tue_Apr_14_11_16_47_2015.txt',
'data1Tue_Apr_14_11_17_05_2015.txt',
'data1Tue_Apr_14_11_17_22_2015.txt',
'data1Tue_Apr_14_11_17_40_2015.txt',
'data1Tue_Apr_14_11_17_57_2015.txt',
'data1Tue_Apr_14_11_18_15_2015.txt',
'data1Tue_Apr_14_11_18_33_2015.txt',
'data1Tue_Apr_14_11_18_50_2015.txt',
'data1Tue_Apr_14_11_19_08_2015.txt',
'data1Tue_Apr_14_11_19_26_2015.txt',
'data1Tue_Apr_14_11_19_44_2015.txt',
'data1Tue_Apr_14_11_20_01_2015.txt',
'data1Tue_Apr_14_11_20_19_2015.txt',
'data1Tue_Apr_14_11_20_37_2015.txt',
'data1Tue_Apr_14_11_20_55_2015.txt',
'data1Tue_Apr_14_11_21_12_2015.txt',
'data1Tue_Apr_14_11_21_30_2015.txt',
'data1Tue_Apr_14_11_21_47_2015.txt',
'data1Tue_Apr_14_11_22_05_2015.txt',
'data1Tue_Apr_14_11_22_23_2015.txt',
'data1Tue_Apr_14_11_22_41_2015.txt',
'data1Tue_Apr_14_11_22_58_2015.txt',
'data1Tue_Apr_14_11_23_16_2015.txt',
'data1Tue_Apr_14_11_23_33_2015.txt',
'data1Tue_Apr_14_11_23_51_2015.txt',
'data1Tue_Apr_14_11_24_09_2015.txt',
'data1Tue_Apr_14_11_24_26_2015.txt',
'data1Tue_Apr_14_11_24_44_2015.txt',
'data1Tue_Apr_14_11_25_02_2015.txt',
'data1Tue_Apr_14_11_25_20_2015.txt',
'data2Tue_Apr_14_11_25_39_2015.txt',
'data2Tue_Apr_14_11_25_56_2015.txt',
'data2Tue_Apr_14_11_26_14_2015.txt',
'data2Tue_Apr_14_11_26_32_2015.txt',
'data2Tue_Apr_14_11_26_49_2015.txt',
'data2Tue_Apr_14_11_27_07_2015.txt',
'data2Tue_Apr_14_11_27_25_2015.txt',
'data2Tue_Apr_14_11_27_42_2015.txt',
'data2Tue_Apr_14_11_27_59_2015.txt',
'data2Tue_Apr_14_11_28_18_2015.txt',
'data2Tue_Apr_14_11_28_35_2015.txt',
'data2Tue_Apr_14_11_28_52_2015.txt',
'data2Tue_Apr_14_11_29_10_2015.txt',
'data2Tue_Apr_14_11_29_27_2015.txt',
'data2Tue_Apr_14_11_29_45_2015.txt',
'data2Tue_Apr_14_11_30_02_2015.txt',
'data2Tue_Apr_14_11_30_19_2015.txt',
'data2Tue_Apr_14_11_30_37_2015.txt',
'data2Tue_Apr_14_11_30_55_2015.txt',
'data2Tue_Apr_14_11_31_13_2015.txt',
'data2Tue_Apr_14_11_31_31_2015.txt',
'data2Tue_Apr_14_11_31_49_2015.txt',
'data2Tue_Apr_14_11_32_07_2015.txt',
'data2Tue_Apr_14_11_32_24_2015.txt',
'data2Tue_Apr_14_11_32_42_2015.txt',
'data2Tue_Apr_14_11_32_59_2015.txt',
'data2Tue_Apr_14_11_33_17_2015.txt',
'data2Tue_Apr_14_11_33_35_2015.txt',
'data2Tue_Apr_14_11_33_53_2015.txt',
'data2Tue_Apr_14_11_34_11_2015.txt',
'data2Tue_Apr_14_11_34_28_2015.txt',
'data2Tue_Apr_14_11_34_46_2015.txt',
'data2Tue_Apr_14_11_35_03_2015.txt',
'data2Tue_Apr_14_11_35_21_2015.txt',
'data2Tue_Apr_14_11_35_39_2015.txt',
'data2Tue_Apr_14_11_35_56_2015.txt',
'data2Tue_Apr_14_11_36_14_2015.txt',
'data2Tue_Apr_14_11_36_32_2015.txt',
'data2Tue_Apr_14_11_36_50_2015.txt',
'data2Tue_Apr_14_11_37_07_2015.txt',
'data2Tue_Apr_14_11_37_25_2015.txt',
'data2Tue_Apr_14_11_37_42_2015.txt',
'data2Tue_Apr_14_11_38_00_2015.txt',
'data2Tue_Apr_14_11_38_17_2015.txt',
'data2Tue_Apr_14_11_38_35_2015.txt',
'data2Tue_Apr_14_11_38_52_2015.txt',
'data2Tue_Apr_14_11_39_09_2015.txt',
'data2Tue_Apr_14_11_39_27_2015.txt',
'data2Tue_Apr_14_11_39_45_2015.txt',
'data2Tue_Apr_14_11_40_03_2015.txt',
'data3Tue_Apr_14_11_40_20_2015.txt',
'data3Tue_Apr_14_11_40_39_2015.txt',
'data3Tue_Apr_14_11_40_56_2015.txt',
'data3Tue_Apr_14_11_41_14_2015.txt',
'data3Tue_Apr_14_11_41_31_2015.txt',
'data3Tue_Apr_14_11_41_49_2015.txt',
'data3Tue_Apr_14_11_42_07_2015.txt',
'data3Tue_Apr_14_11_42_25_2015.txt',
'data3Tue_Apr_14_11_42_43_2015.txt',
'data3Tue_Apr_14_11_43_01_2015.txt',
'data3Tue_Apr_14_11_43_18_2015.txt',
'data3Tue_Apr_14_11_43_36_2015.txt',
'data3Tue_Apr_14_11_43_54_2015.txt',
'data3Tue_Apr_14_11_44_11_2015.txt',
'data3Tue_Apr_14_11_44_29_2015.txt',
'data3Tue_Apr_14_11_44_45_2015.txt',
'data3Tue_Apr_14_11_45_04_2015.txt',
'data3Tue_Apr_14_11_45_22_2015.txt',
'data3Tue_Apr_14_11_45_39_2015.txt',
'data3Tue_Apr_14_11_45_57_2015.txt',
'data3Tue_Apr_14_11_46_14_2015.txt',
'data3Tue_Apr_14_11_46_32_2015.txt',
'data3Tue_Apr_14_11_46_50_2015.txt',
'data3Tue_Apr_14_11_47_08_2015.txt',
'data3Tue_Apr_14_11_47_26_2015.txt',
'data3Tue_Apr_14_11_47_44_2015.txt',
'data3Tue_Apr_14_11_48_02_2015.txt',
'data3Tue_Apr_14_11_48_20_2015.txt',
'data3Tue_Apr_14_11_48_38_2015.txt',
'data3Tue_Apr_14_11_48_56_2015.txt',
'data3Tue_Apr_14_11_49_14_2015.txt',
'data3Tue_Apr_14_11_49_32_2015.txt',
'data3Tue_Apr_14_11_49_49_2015.txt',
'data3Tue_Apr_14_11_50_07_2015.txt',
'data3Tue_Apr_14_11_50_25_2015.txt',
'data3Tue_Apr_14_11_50_42_2015.txt',
'data3Tue_Apr_14_11_51_00_2015.txt',
'data3Tue_Apr_14_11_51_17_2015.txt',
'data3Tue_Apr_14_11_51_35_2015.txt',
'data3Tue_Apr_14_11_51_52_2015.txt',
'data3Tue_Apr_14_11_52_10_2015.txt',
'data3Tue_Apr_14_11_52_27_2015.txt',
'data3Tue_Apr_14_11_52_45_2015.txt',
'data3Tue_Apr_14_11_53_02_2015.txt',
'data3Tue_Apr_14_11_53_21_2015.txt',
'data3Tue_Apr_14_11_53_39_2015.txt',
'data3Tue_Apr_14_11_53_58_2015.txt',
'data3Tue_Apr_14_11_54_16_2015.txt',
'data3Tue_Apr_14_11_54_34_2015.txt',
'data3Tue_Apr_14_11_54_51_2015.txt',
'data4Tue_Apr_14_12_38_13_2015.txt',
'data4Tue_Apr_14_12_38_31_2015.txt',
'data4Tue_Apr_14_12_38_50_2015.txt',
'data4Tue_Apr_14_12_39_08_2015.txt',
'data4Tue_Apr_14_12_39_26_2015.txt',
'data4Tue_Apr_14_12_39_45_2015.txt',
'data4Tue_Apr_14_12_40_03_2015.txt',
'data4Tue_Apr_14_12_40_20_2015.txt',
'data4Tue_Apr_14_12_40_38_2015.txt',
'data4Tue_Apr_14_12_40_56_2015.txt',
'data4Tue_Apr_14_12_41_14_2015.txt',
'data4Tue_Apr_14_12_41_32_2015.txt',
'data4Tue_Apr_14_12_41_50_2015.txt',
'data4Tue_Apr_14_12_42_08_2015.txt',
'data4Tue_Apr_14_12_42_25_2015.txt',
'data4Tue_Apr_14_12_42_44_2015.txt',
'data4Tue_Apr_14_12_43_02_2015.txt',
'data4Tue_Apr_14_12_43_21_2015.txt',
'data4Tue_Apr_14_12_43_39_2015.txt',
'data4Tue_Apr_14_12_43_57_2015.txt',
'data4Tue_Apr_14_12_44_15_2015.txt',
'data4Tue_Apr_14_12_44_34_2015.txt',
'data4Tue_Apr_14_12_44_52_2015.txt',
'data4Tue_Apr_14_12_45_10_2015.txt',
'data4Tue_Apr_14_12_45_28_2015.txt',
'data4Tue_Apr_14_12_45_47_2015.txt',
'data4Tue_Apr_14_12_46_05_2015.txt',
'data4Tue_Apr_14_12_46_23_2015.txt',
'data4Tue_Apr_14_12_46_41_2015.txt',
'data4Tue_Apr_14_12_47_00_2015.txt',
'data4Tue_Apr_14_12_47_17_2015.txt',
'data4Tue_Apr_14_12_47_35_2015.txt',
'data4Tue_Apr_14_12_47_54_2015.txt',
'data4Tue_Apr_14_12_48_12_2015.txt',
'data4Tue_Apr_14_12_48_30_2015.txt',
'data4Tue_Apr_14_12_48_48_2015.txt',
'data4Tue_Apr_14_12_49_07_2015.txt',
'data4Tue_Apr_14_12_49_24_2015.txt',
'data4Tue_Apr_14_12_49_42_2015.txt',
'data4Tue_Apr_14_12_50_01_2015.txt',
'data4Tue_Apr_14_12_50_18_2015.txt',
'data4Tue_Apr_14_12_50_37_2015.txt',
'data4Tue_Apr_14_12_50_55_2015.txt',
'data4Tue_Apr_14_12_51_13_2015.txt',
'data4Tue_Apr_14_12_51_31_2015.txt',
'data4Tue_Apr_14_12_51_49_2015.txt',
'data4Tue_Apr_14_12_52_08_2015.txt',
'data4Tue_Apr_14_12_52_26_2015.txt',
'data4Tue_Apr_14_12_52_45_2015.txt',
'data4Tue_Apr_14_12_53_03_2015.txt']
