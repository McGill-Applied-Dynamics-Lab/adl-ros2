import pandas as pd
import numpy as np

def multi_user_data():

    nb_users = 30 # Number of users
    nb_maneuvers = 2 # Number of maneuvers (guided, free)
    nb_communcation_tasks = 1 # 5G
    nb_latency_tasks = 3 # 0-ZOH, 1-ZOHPhi, 2-DelayRIM

    df1 = pd.DataFrame({})

    np.random.seed(30)

    for user in range(0,nb_users):

        for maneuver in range(0,nb_maneuvers):

            task = 0
            random_task = np.random.choice(nb_latency_tasks * nb_communcation_tasks,
                                           nb_latency_tasks * nb_communcation_tasks, replace=False)
            for task in range(0, nb_latency_tasks*nb_communcation_tasks):
            # Choose a random order for the communication model

                task_latency = random_task[task] % nb_latency_tasks
                task_communication = random_task[task]//nb_latency_tasks

                df2 = pd.DataFrame({'User':[user],'Maneuver':[maneuver],'Task':[task],
                    'CommunicationModel':[task_communication],'LatencyModel':[task_latency]})

                df1 = df1.append(df2, ignore_index=True)

    return df1

multi_user_df = multi_user_data()
multi_user_df.to_csv('multi_user_df.csv', sep=',', index=False)