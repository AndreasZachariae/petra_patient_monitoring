#!/usr/bin/env python3
import numpy as np
import pandas as pd
import random
import json

############################################# Konstanten ################################################################
path = "~/petra_ws/src/petra_patient_monitoring/data/features_version_01.csv"
purity_score = "entropy"  # Alternative: "gini"
training_percentage = 0.8
gain_threshold = 0  # Gain-Wert unter dem keine weiteren Äste gebildet werden
header = ["num", "Image", "Video", "Frame", "Class", "Presence", "TorsoBoundingBoxRatio", "HeadGroundDistance", "BufferedHeadGroundDistance",
          "HeadVelocity", "BufferedHeadVelocity", "TorsoHeight", "BufferedTorsoHeight", "Centroid", "BufferedCentroid"]
first_feature_column = 5
number_of_features = 10
column_of_label = 4
allow_same_feature_again = False
#########################################################################################################################

# Speichere Daten in Array, wobei jede Zeile eine Liste beinhaltet in Form des headers
features = list(range(first_feature_column, first_feature_column + number_of_features))  # [5,6,7,8,9,10,11,12,13,14] len=10

df = pd.read_csv(path, header=0)
data = []
for row in range(len(df)):
    row_data = []
    for col in range(len(header)):
        row_data.append(df.iloc[row, col])
    data.append(row_data)


# Gesamten Datensatz zufällig mischen, 80% davon zum Training verwenden und 20% als Testdaten aufheben
random.shuffle(data)
split_value = len(data) * training_percentage
split_index = int(round(split_value))
training_data = data[:split_index]
testing_data = data[split_index:]


# Trainings- und Testdatensatz in der Konsole ausgeben
print("Gesamter Datensatz enthält "+str(len(data))+" Elemente")
print("Trainingsdatensatz enthält "+str(len(training_data))+" Elemente")
# for elem in range(len(training_data)):
#    print(training_data[elem])
# print("")
print("Testdatensatz enthält "+str(len(testing_data))+" Elemente")


# Zählt die Elemente pro Klasse und gibt ein dictionary mit ('label': count) zurück
def count_per_label(rows):
    counts = {}
    for row in rows:
        label = row[column_of_label]
        if label not in counts:
            counts[label] = 0
        counts[label] += 1
    return counts


# Berechnet die 'Entropie' (Unreinheit eines Datensatzen)
# 0 = nur Daten einer Klasse vorhanden, 1 = gleich viele Daten von jeder Klasse vorhanden
def entropy(rows):
    entropy = 0
    counts = count_per_label(rows)
    for label in counts:
        prob = counts[label] / float(len(rows))
        entropy += (-1 * prob) * np.log2(prob)
    return entropy


# Berechnet die 'Gini Impurity' für einen gegebenen Datensatz
# 0 = 'reiner' Datensätze, 0.5 = max. 'unreiner' (d.h. durchmischter) Datensatz
def gini(rows):
    counts = count_per_label(rows)
    impurity = 1
    for label in counts:
        prob = counts[label] / float(len(rows))
        impurity -= prob**2
    return impurity


# Benutze gewünschte Reinheitsgrad-Formel
def purity(rows):
    current_purity = 0
    if (purity_score == "gini"):
        current_purity = gini(rows)
    elif (purity_score == "entropy"):
        current_purity = entropy(rows)
    else:
        print("unknown purity score")
    return current_purity


# Berechnung des 'Information Gain'
def gain(true_rows, false_rows, current_purity):
    p = float(len(true_rows)) / (len(true_rows) + len(false_rows))
    return current_purity - p * purity(true_rows) - (1 - p) * purity(false_rows)


# Entscheidungsfunktion
def descision(row, feature, value):
    return row[feature] >= value


# Teilt Datensatz anhand des gegebenen Merkmal und Wertes
# Merkmal >= value ist true, Merkmal < value = false
def split(rows, feature, value):
    true_rows, false_rows = [], []
    for row in rows:
        if descision(row, feature, value):
            true_rows.append(row)
        else:
            false_rows.append(row)
    return true_rows, false_rows


# Finde das beste Entscheidungskriterium für die aktuelle Stelle des Baums durch Iteration über jedes Merkmal und jeden Wert des Merkmals
# Berechnung des 'Information Gain' für jede mögliche Entscheidung
def find_best_split(rows, features):
    best_gain = 0
    best_feature = 0
    best_value = 0
    current_purity = purity(rows)

    for feature in features:  # Iteration über jedes noch nicht verwendete Merkmal

        values = set([row[feature] for row in rows])  # einzigartige Werte im Datensatz

        for value in values:  # Iteration über jeden (Trainings)-Wert eines Merkmals

            # Teile den Datensatz anhand des Entscheidungskriteriums
            true_rows, false_rows = split(rows, feature, value)

            # Abfrage, ob der Datensatz überhaupt geteilt wird
            if len(true_rows) == 0 or len(false_rows) == 0:
                continue

            # Berechne 'Information Gain' für aktuellen Durchlauf
            current_gain = gain(true_rows, false_rows, current_purity)

            # Speichere den besten gefundenen Gain und das zugehörige Entscheidungskriterium
            if current_gain >= best_gain:
                best_feature, best_value, best_gain = feature, value, current_gain

    return best_feature, best_value, best_gain


# Blatt am Ende des Baums
class Leaf:
    def __init__(self, rows):
        self.counts = count_per_label(rows)
        self.rows = rows
        self.prediction = ["undecidable", 0]
        sum = 0

        for key, value in self.counts.items():
            sum += value
            if value > self.prediction[1]:
                self.prediction = [key, value]

        self.prediction[1] = round(self.prediction[1] / float(sum), 3)


# Entscheidungsknoten
class Node:
    def __init__(self, feature, value, gain, true_branch, false_branch):
        self.feature = feature
        self.value = value
        self.gain = gain
        self.true_branch = true_branch
        self.false_branch = false_branch


# Aufbau des Entscheidungsbaums
def build_tree(rows, features):

    # Finde das bestmögliche Entscheidungskriterium
    feature, value, gain = find_best_split(rows, features)

    # Kein weiterer Informationsgewinn durch aufteilen des Baums, d.h. Baum ist am Ende (Blatt)
    if gain <= gain_threshold:
        return Leaf(rows)

    # Teile den Datensatz anhand des gefundenen Entscheidungskriteriums
    true_rows, false_rows = split(rows, feature, value)

    # Verwendetes Merkmal aus der Liste entfernen
    if(allow_same_feature_again == False):
        features.remove(feature)

    # Alle Merkmale wurden verwendet
    if len(features) == 0:
        return Leaf(rows)

    # Erstelle den 'True'-Ast der Entscheidung rekursiv
    true_branch = build_tree(true_rows, features)

    # Erstelle den 'False'-Ast der Entscheidung rekursiv
    false_branch = build_tree(false_rows, features)

    # Node speichert das Entscheidungskriterium und die zu verfolgenden Äste (abhängig von der Antwort auf die Frage)
    return Node(feature, value, gain, true_branch, false_branch)


# Funktion zur Ausgabe des Entscheidungsbaums in der Konsole
def print_tree(node, spacing=""):

    # Blatt (Ende des Baums, d.h. Klassifikation)
    if isinstance(node, Leaf):
        print(spacing + "Counts per class " + str(node.counts) + " Prediction " + str(node.prediction))
        return

    # Entscheidungskriterium
    print(spacing + "Is " + str(header[node.feature]) + " >= " + str(round(node.value, 3)) + " (gain: " + str(round(node.gain, 3)) + ")")

    # True-Ast
    print(spacing + '--> True:')
    print_tree(node.true_branch, spacing + "    ")

    # False-Ast
    print(spacing + '--> False:')
    print_tree(node.false_branch, spacing + "    ")


# Klassifizieren von Test-Daten
def classify(row, node):

    # Klassifikation, wenn an Blatt angekommen
    if isinstance(node, Leaf):
        return node.prediction

    # Entscheidung, welcher Ast des Baums weiterverfolgt wird (anhand des Entscheidungskriteriums)
    if descision(row, node.feature, node.value):
        return classify(row, node.true_branch)
    else:
        return classify(row, node.false_branch)


def evaluate(tree):
    correct = 0

    for row in testing_data:
        prediction = classify(row, tree)
        if row[column_of_label] == prediction[0]:
            correct += 1

    print("Evaluation: " + str(round(correct/len(testing_data) * 100)) + "% (" + str(correct) + "/" + str(len(testing_data)) + ") correct predicted")


def serialize(obj):
    if isinstance(obj, Node):
        return {"feature": obj.feature,
                "value": obj.value,
                "gain": obj.gain,
                "true_branch": serialize(obj.true_branch),
                "false_branch": serialize(obj.false_branch)}

    if isinstance(obj, Leaf):
        return [str(obj.prediction[0]), obj.prediction[1]]

    return "unknown obj"


# main-Funktion
if __name__ == '__main__':

    print(count_per_label(training_data))

    descision_tree = build_tree(training_data, features)

    print_tree(descision_tree)

    evaluate(descision_tree)

    #json_tree = json.dumps(descision_tree, default=serialize, indent=4)

    with open('tree.json', 'w') as file:
        json.dump(descision_tree, file, default=serialize, indent=4)

    print("")
    print("Teste das Bild " + str(data[6]))
    print("Ergebnis:", classify(data[6], descision_tree))
