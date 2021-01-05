#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# read features from .csv file
data_path = "~/petra_ws/src/petra_patient_monitoring/data/features_version_01.csv"
df = pd.read_csv(data_path, header=0)
df_walk = df.loc[df.Class == 0]  # 0 = walking
df_fall = df.loc[df.Class == 1]  # 1 = falling

print("number of walking poses: ", df_walk.shape[0])
print("number of falling poses: ", df_fall.shape[0])


def create_sublot(axis, x_label, y_label, x_plot, y_plot):
    axis[x_plot, y_plot].scatter(df_walk[x_label], df_walk[y_label], s=5, color='blue')
    axis[x_plot, y_plot].scatter(df_fall[x_label], df_fall[y_label], s=5, color='red')
    #axis[x_plot, y_plot].set_title(x_label + ' vs. ' + y_label)
    axis[x_plot, y_plot].set_xlabel(x_label)
    axis[x_plot, y_plot].set_ylabel(y_label)


# compare buffered vs unbuffered
fig1, axs1 = plt.subplots(2, 4)

create_sublot(axs1, "TorsoBoundingBoxRatio", "HeadGroundDistance", 0, 0)
create_sublot(axs1, "TorsoBoundingBoxRatio", "BufferedHeadGroundDistance", 1, 0)
create_sublot(axs1, "TorsoBoundingBoxRatio", "HeadVelocity", 0, 1)
create_sublot(axs1, "TorsoBoundingBoxRatio", "BufferedHeadVelocity", 1, 1)
create_sublot(axs1, "TorsoBoundingBoxRatio", "TorsoHeight", 0, 2)
create_sublot(axs1, "TorsoBoundingBoxRatio", "BufferedTorsoHeight", 1, 2)
create_sublot(axs1, "TorsoBoundingBoxRatio", "Centroid", 0, 3)
create_sublot(axs1, "TorsoBoundingBoxRatio", "BufferedCentroid", 1, 3)

plt.show()


fig2, axs2 = plt.subplots(2, 4)

create_sublot(axs2, "Presence", "HeadGroundDistance", 0, 0)
create_sublot(axs2, "Presence", "BufferedHeadGroundDistance", 1, 0)
create_sublot(axs2, "Presence", "HeadVelocity", 0, 1)
create_sublot(axs2, "Presence", "BufferedHeadVelocity", 1, 1)
create_sublot(axs2, "Presence", "TorsoHeight", 0, 2)
create_sublot(axs2, "Presence", "BufferedTorsoHeight", 1, 2)
create_sublot(axs2, "Presence", "Centroid", 0, 3)
create_sublot(axs2, "Presence", "BufferedCentroid", 1, 3)

plt.show()


# show best seperators
fig3, axs3 = plt.subplots(2, 2)

create_sublot(axs3, "TorsoBoundingBoxRatio", "Presence", 0, 0)
create_sublot(axs3, "HeadVelocity", "HeadGroundDistance", 0, 1)
create_sublot(axs3, "TorsoBoundingBoxRatio", "HeadVelocity", 1, 0)
create_sublot(axs3, "TorsoBoundingBoxRatio", "HeadGroundDistance", 1, 1)

plt.show()
