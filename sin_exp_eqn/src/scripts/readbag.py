import bagpy
from bagpy import bagreader
import pandas as pd

b = bagreader('sin_exp_data.bag')

print(b.topic_table)

sin_topic_csv = b.message_by_topic('/sin_topic')
exp_topic_csv = b.message_by_topic('/exp_topic')
calculated_y_csv = b.message_by_topic('/calculated_y')

df_sin = pd.read_csv(sin_topic_csv)
df_exp = pd.read_csv(exp_topic_csv)
df_y = pd.read_csv(calculated_y_csv)

# Print the first 5 rows of the dataframe
print('df_sin',df_sin.head())
print('df_exp',df_exp.head())
print('df_y',df_y.head())

