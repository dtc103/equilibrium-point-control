from typing_extensions import runtime
import pandas as pd
from pandas.core.frame import DataFrame
import seaborn as sns
import matplotlib.pyplot as plt

sns.set()


sm_bf = pd.read_csv("step_size_brute_force_biceps_curl.csv", sep=";")
sm_ps = pd.read_csv("step_size_pattern_search_biceps_curl.csv", sep=";")

sm_bf["accuracy"] = (abs(sm_bf["elbow_jointangle_desired"] - sm_bf["elbow_jointangle_final"]))  / sm_bf["elbow_jointangle_desired"]
sm_ps["accuracy"] = (abs(sm_ps["elbow_jointangle_desired"] - sm_ps["elbow_jointangle_final"]))  / sm_ps["elbow_jointangle_desired"]

arm26_bf = pd.read_csv("execution_time_brute_force_simple_model_deterministic_arm26_with_cache.csv", sep=";")
arm26_ps = pd.read_csv("execution_time_pattern_search_simple_model_arm26.csv", sep=";")

arm26_ps["accuracy"] = ((abs(arm26_ps["r_shoulder_jointangle_desired"] - arm26_ps["r_shoulder_jointangle_final"])) + (abs(arm26_ps["r_elbow_jointangle_desired"] - arm26_ps["r_elbow_jointangle_final"]))) / (arm26_ps["r_elbow_jointangle_desired"] + arm26_ps["r_shoulder_jointangle_desired"])
arm26_bf["accuracy"] = ((abs(arm26_bf["r_shoulder_jointangle_desired"] - arm26_bf["r_shoulder_jointangle_final"])) + (abs(arm26_bf["r_elbow_jointangle_desired"] - arm26_bf["r_elbow_jointangle_final"]))) / (arm26_bf["r_elbow_jointangle_desired"] + arm26_bf["r_shoulder_jointangle_desired"])





#step_size_sm_bf = pd.read_csv("step_size_brute_force_biceps_curl.csv", sep=";")
#step_size_sm_ps = pd.read_csv("step_size_pattern_search_biceps_curl.csv", sep=";")



fig = sns.lineplot(data = sm_bf, x = "gradient_stepsize", y = "accuracy", label="BF")
fig = sns.lineplot(data = sm_ps, x = "gradient_stepsize", y = "accuracy", label="PS" )

fig.set_title("step size influence on error\nSimple Model (SM)")
fig.set(xlabel="step size", ylabel="relative error")

plt.show()

#step_size_arm26_bf = pd.read_csv("step_size_brute_force_arm26.csv", sep=";")
#step_size_arm26_ps = pd.read_csv("step_size_pattern_search_arm26.csv", sep=";")
#step_size_arm26_ps["accuracy"] = ((abs(step_size_arm26_ps["r_shoulder_jointangle_desired"] - step_size_arm26_ps["r_shoulder_jointangle_final"])) + (abs(step_size_arm26_ps["r_elbow_jointangle_desired"] - step_size_arm26_ps["r_elbow_jointangle_final"]))) / (step_size_arm26_ps["r_elbow_jointangle_desired"] + step_size_arm26_ps["r_shoulder_jointangle_desired"])
#step_size_arm26_bf["accuracy"] = ((abs(step_size_arm26_bf["r_shoulder_jointangle_desired"] - step_size_arm26_bf["r_shoulder_jointangle_final"])) + (abs(step_size_arm26_bf["r_elbow_jointangle_desired"] - step_size_arm26_bf["r_elbow_jointangle_final"]))) / (step_size_arm26_bf["r_elbow_jointangle_desired"] + step_size_arm26_bf["r_shoulder_jointangle_desired"])

#fig = sns.lineplot(data = step_size_arm26_bf, x = "gradient_stepsize", y = "accuracy", label="BF")
#fig = sns.lineplot(data = step_size_arm26_ps, x = "gradient_stepsize", y = "accuracy", label="PS" )

#fig.set_title("step size influence on accuracy\nArm26")
#fig.set(xlabel="step size", ylabel="relative error")

#plt.show()



# goal_angles_bf = pd.read_csv("goal_angle_distance_brute_force_simple_model_biceps_curl.csv", sep=";")
# goal_angles_ps = pd.read_csv("goal_angle_distance_pattern_search_simple_model_biceps_curl.csv", sep=";")


# fig = sns.lineplot(data = goal_angles_bf, x = "elbow_jointangle_desired", y = "execution_time", label="BF")
# fig = sns.lineplot(data = goal_angles_ps, x = "elbow_jointangle_desired", y = "execution_time", label="PS")
# fig.set_title("goal angle change\n step size = 0.001")
# fig.set(xlabel="goal_angle", ylabel="execution time [ms]")

# plt.show()