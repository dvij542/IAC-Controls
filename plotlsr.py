import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('test.txt',delimiter = ',')
# data1 = np.loadtxt('test2.txt',delimiter = ',')
# plt.plot(data[5,:])
# plt.plot(data[2,:])
# plt.plot(data[2,:])
# plt.plot((data[5,:])*100)#+data[5,:]+data[6,:]))
# plt.plot((data[7,:]+data[8,:]+data[9,:]+data[10,:])/1000)
# plt.plot(data[2,:]/100)
# plt.plot(data[-2,:]/100)
# plt.plot(data[-1,:])
# plt.plot(data1[-1,:])
plt.plot((data[5,:]))#+data[4,:]+data[5,:]+data[6,:]))
plt.plot((data[6,:]))#+data[4,:]+data[5,:]+data[6,:]))
# plt.plot((data1[7,:]+data1[8,:]+data1[9,:]+data1[10,:]))
# data = np.loadtxt('test_start4.txt',delimiter = ',')
# plt.plot(data[5,:])
# plt.plot(data[2,:])

# data = np.loadtxt('test_start10.txt',delimiter = ',')
# plt.plot(data[5,:])
# plt.plot(data[2,:])
# # print(data[5,:])
# plt.plot(data[7,:]+data[8,:]+data[9,:]+data[10,:])#+data[9,:]+data[10,:])
# plt.plot(1000*data[2,:])

# plt.plot(data[11,:]+data[12,:]+data[13,:]+data[14,:])

# data = np.loadtxt('test3.txt',delimiter = ',')
# plt.plot(data[5,:])
# data2 = np.loadtxt('test2.txt',delimiter = ',')
# plt.plot(data2[5,:])
# # plt.plot(data2[2,:])
# data3 = np.loadtxt('test.txt',delimiter = ',')
# plt.plot(data3[5,:])
# # plt.plot(data3[2,:])

plt.show()

