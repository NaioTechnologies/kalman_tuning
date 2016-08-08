# SensorFusion

Simulateur python pour régler le filtre de kalman étendu.
Permet de visualiser les différentes odométries (fusion, meca, visu, gps...)

## Modules python à installer

- matplotlib pour l'affichage:
```
sudo apt-get install python-matplotlib
```
- numpy pour les calculs matriciels:
```
sudo apt-get install python-numpy
```

## Utilisation

**parser.py** récupère les mesures d'un fichier log_lvl2 (par défaut) et les stocke dans un fichier excel file.csv.
```
python parser.py chemin_log
```
**sensor_fusion_tuning.py** récupère les données de file.csv (par défaut), rejoue le log et affiche la position obtenue par fusion.
```
Usage: python sensor_fusion_tuning.py chemin_csv
```
