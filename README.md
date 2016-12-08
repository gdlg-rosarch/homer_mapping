# nav_libs

## Known Issues / Todo's 

Im Package nav_libs ist eine Kopie der Robbie-Bibliothek Math, umbenannt in MappingMath, die nur von homer_mapping benutzt werden soll, solange es keine vernünftige Alternative für die Pose gibt.

## Introduction 

Das Package nav_libs enthält einige Bibliotheken, die vom Package homer_mapping und nav_libs verwendet werden. Außerdem enthält es im Ordner tools die Header-Datei tools.h, die Funktionen zum Transformieren in verschiedene Koordinatenframes enthält. All diese Funktionen befinden sich im namespace "map_tools".

* Die Bibliothek `Explorer` wird von homer_mapping und homer_navigation verwendet und enthält die Pfadplanungsalgorithmen A-Stern sowie die dafür benötigte Datenstruktur der GridMap.
* Die Bibliothek `SpeedControl` wird von homer_navigation verwendet und  ist dafür zuständig abhängig von den aktuellen Laserdaten die höchstzulässige Geschwindigkeit zu berechnen.
* Die Bibliothek `MappingMath` wird von homer_mapping verwendet und enthält die Datenstruktur Pose, in der die aktuelle Roboterposition innerhalb der Node gespeichert wird.
