"""
This file will take the 1st row of a CSV file as fields, and optionally all or all but the first columns as data for plotting. We can plot up to 7 columns
"""
import matplotlib.pyplot as plt
import csv
import argparse

parser = argparse.ArgumentParser(description='Usage: python3 -f CSV_FILE plot_csv.py')
parser.add_argument('-f', "--file", default='', help='csv file name')
parser.add_argument('-x', "--provide_x", default=False, action='store_true', help='set to true if x axis data is the first column of the csv file')

args, unknown = parser.parse_known_args()

csv_file_name = args.file
provide_x = args.provide_x 
# first row must be the x axis values
data = []
fields = []
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

with open(csv_file_name,'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    fields = next(lines)
    if not provide_x:
        fields.insert(0, "number of samples")
    [data.append([]) for i in fields]
    linenum = 0
    for row in lines:
        # x_data
        data[0].append(row[0] if provide_x else linenum)
        for i in range(1, len(fields)):
            data[i].append(float(row[i]) if provide_x else float(row[i-1]))
        linenum += 1


    for i in range(1, len(fields)):
        plt.plot(data[0], data[i], color = colors[i-1], linestyle = 'dashed', marker = 'o',label = fields[i])

print("Plotting CSV: ")
print(f"Fields: {fields}")

# plt.xticks(rotation = 25)
plt.xlabel(fields[0])
plt.ylabel("value")
plt.title('Report', fontsize = 20)
# plt.grid()
plt.legend()
plt.show()
