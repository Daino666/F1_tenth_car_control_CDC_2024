import matplotlib.pyplot as plt
import pandas as pd
from io import StringIO

# Given data
data = """
positions_X,positions_y,
0.7411,3.1583
0.7411,3.1583
0.7411,3.1564
0.7411,3.1246
0.7416,3.0553
0.7427,2.9815
0.7468,2.8274
0.7533,2.6764
0.7632,2.5158
0.7763,2.3527
0.8017,2.1087
0.8237,1.9386
0.8488,1.7729
0.8735,1.6275
0.9016,1.4765
0.9316,1.326
0.9627,1.1756
0.9947,1.0214
1.0204,0.8864
1.0416,0.7461
1.0543,0.598
1.0688,0.4595
1.0881,0.3135
1.1163,0.1467
1.1474,-0.0013
1.182,-0.1403
1.2211,-0.2761
1.265,-0.4104
1.3143,-0.5449
1.4005,-0.7504
1.499,-0.9525
1.5987,-1.1315
1.7238,-1.3278
1.8607,-1.516
1.9867,-1.6699
2.1425,-1.8402
2.2897,-1.986
2.4303,-2.1157
2.5835,-2.2524
2.7243,-2.3863
2.8523,-2.5385
2.926,-2.6503
3.0266,-2.8255
3.1284,-3.0408
3.2098,-3.2378
3.2633,-3.371
3.3322,-3.5453
3.3833,-3.6751
3.4321,-3.7995
3.4735,-3.9054
3.551,-4.1041
3.6225,-4.2881
3.7068,-4.5056
3.7821,-4.7009
3.8614,-4.9076
3.9446,-5.1255
4.0267,-5.3417
4.0768,-5.4741
4.1204,-5.5898
4.1703,-5.7223
4.2179,-5.8492
4.2654,-5.9762
4.3051,-6.0827
4.3469,-6.1948
4.3913,-6.3144
4.4406,-6.4471
4.4884,-6.5761
4.534,-6.6995
4.5747,-6.8099
4.6154,-6.9203
4.665,-7.055
4.7131,-7.186
4.7606,-7.3151
4.8075,-7.4423
4.8573,-7.5693
4.912,-7.682
4.9799,-7.7965
5.055,-7.9017
5.1267,-7.9872
4.9795,-6.0078
4.9805,-6.0486
4.9845,-6.1307
4.9883,-6.29
4.9848,-6.4398
4.9734,-6.5849
4.9561,-6.7062
4.9257,-6.849
4.8865,-6.9836
4.844,-7.1005
4.7866,-7.2335
4.7402,-7.3326
4.6921,-7.4431
4.6477,-7.56
4.5895,-7.6986
4.5304,-7.8195
4.4339,-7.9853
4.298,-8.1749
4.1677,-8.3236
4.0258,-8.4582
3.8658,-8.584
3.7048,-8.688
4.4091,-8.1366
4.3657,-8.1437
4.267,-8.1578
4.1189,-8.172
3.9273,-8.1725
3.8121,-8.1631
3.7206,-8.1499
3.6222,-8.1301
3.5287,-8.1059
3.4349,-8.076
3.374,-8.0535
3.2619,-8.0052
3.174,-7.9608
3.0889,-7.9115
2.9322,-7.8028
2.7826,-7.6722
2.6286,-7.4973
2.5071,-7.3103
2.4247,-7.1332
2.3572,-6.9464
2.2977,-6.7564
2.2454,-6.5642
2.199,-6.3663
2.1609,-6.1748
2.1274,-5.9703
2.103,-5.7807
2.0836,-5.5764
2.0717,-5.3815
2.0677,-5.264
2.0661,-5.1624
2.0662,-5.0607
2.0677,-4.9691
2.0748,-4.7839
2.0882,-4.5871
2.1084,-4.3869
3.1241,-7.0978
3.0782,-7.0495
2.9972,-6.9647
2.8808,-6.8394
2.7558,-6.691
2.6338,-6.5289
2.5245,-6.365
2.4096,-6.1672
2.3156,-5.9786
2.2414,-5.8048
2.1715,-5.6102
2.1176,-5.4272
2.0721,-5.2316
2.0506,-5.1143
2.035,-5.0101
2.016,-4.8361
2.0048,-4.6435
2.0044,-4.4408
3.1299,-7.1039
3.0942,-7.0662
3.024,-6.992
2.9183,-6.8769
2.7943,-6.7286
2.6569,-6.5436
2.5449,-6.3719
2.4478,-6.2028
2.3537,-6.0141
2.2791,-5.8405
2.2103,-5.6519
2.1545,-5.4672
2.1069,-5.2701
2.085,-5.1569
2.0657,-5.0351
2.0525,-4.9305
2.0368,-4.7462
2.0301,-4.5474
2.0343,-4.3506
3.1172,-7.0907
3.066,-7.0359
2.9814,-6.9438
2.8599,-6.8075
2.7386,-6.6571
2.621,-6.4942
2.5149,-6.3282
2.4146,-6.1495
2.33,-5.9761
2.241,-5.7611
2.1742,-5.5655
2.1227,-5.3796
2.0803,-5.1834
2.0618,-5.0736
2.0483,-4.9751
2.0367,-4.8683
2.0281,-4.7592
2.0208,-4.5624
2.0241,-4.3636
3.1202,-7.0938
3.0708,-7.041
2.9864,-6.9493
2.8689,-6.8175
2.7453,-6.6643
2.63,-6.5047
2.5217,-6.3353
2.4242,-6.1619
2.3368,-5.9832
2.2605,-5.8018
2.1921,-5.6087
2.1378,-5.4216
2.0922,-5.2219
2.0731,-5.1163
2.0583,-5.018
2.0458,-4.9153
2.0363,-4.8143
2.0297,-4.7151
2.0245,-4.5043
2.0309,-4.3015
3.1094,-7.0825
3.0487,-7.0167
2.957,-6.9161
2.8334,-6.7761
2.7089,-6.6197
2.5926,-6.4558
2.4889,-6.2906
2.3776,-6.0862
2.2921,-5.9023
2.2203,-5.7211
2.1551,-5.5228
2.1066,-5.3403
2.0804,-5.2199
2.0621,-5.1201
2.0475,-5.0258
2.0345,-4.9232
2.0239,-4.8143
2.016,-4.6932
2.0117,-4.5163
2.0171,-4.3195
3.1127,-7.086
3.0561,-7.025
2.965,-6.9246
2.8416,-6.7868
2.7184,-6.6347
2.5827,-6.4455
2.4739,-6.2718
2.3803,-6.1009
2.2823,-5.8921
2.2061,-5.6977
2.1475,-5.5182
2.0956,-5.3201
2.069,-5.1937
2.0515,-5.0938
2.0367,-4.9915
2.0256,-4.8966
2.0107,-4.7023
2.0067,-4.5811
2.0085,-4.4022
3.1263,-7.1003



"""

# Reading data into a DataFrame
df = pd.read_csv(StringIO(data))

# Calculate the reference point (midpoint of positions_X and positions_y)
ref_x = df['positions_X'].mean()
ref_y = df['positions_y'].mean()

# Shift the positions relative to the reference point
df['positions_X_relative'] = df['positions_X'] - ref_x
df['positions_y_relative'] = df['positions_y'] - ref_y

# Plotting with data converted to NumPy arrays
plt.figure(figsize=(10, 6))
plt.plot(df['positions_X_relative'].to_numpy(), df['positions_y_relative'].to_numpy(), label='Position (X, Y) relative to Reference Point', marker='o')
plt.scatter(0, 0, color='red', label='Reference Point (0, 0)', marker='x')  # Mark the reference point
plt.xlabel('Position X (relative to Reference Point)')
plt.ylabel('Position Y (relative to Reference Point)')
plt.title('Visualization of Positions Relative to Reference Point (Midpoint)')
plt.legend()
plt.grid(True)
plt.show()
