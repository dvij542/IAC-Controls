import pandas
import matplotlib.pyplot as plt
df = pandas.read_csv("./traj_test.csv", delimiter=';')
print(df.head())
plt.plot(df['# x_ref_m'], df[' y_ref_m'])
plt.savefig('test.png')

plt.show()