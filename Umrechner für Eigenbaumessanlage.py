import csv
x = 0
y = 0
CO2 = 0
CO = 0
NO2 = 0
with open("lolxxx.csv") as csvdatei :
    csv_reader_object = csv.reader(csvdatei, delimiter=";")
    for row in csv_reader_object:
        if y >= 2:
            CO2 += int(row[2])
            CO += int(row[3])
            NO2 += float(row[4])
            x += 1
            if x == 16:
                x = 0
                print(int(CO2/15),";", int(CO/15),";", int(NO2/15))
                CO2 = 0
                CO = 0
                NO2 = 0  
        y += 1