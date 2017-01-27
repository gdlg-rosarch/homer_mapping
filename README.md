# map_messages


## Introduction 

Dieses Package enthält alle benutzerdefinierten Messages, die neben den in ROS enthaltenen Messages für das Mapping und die Navigation verwendet werden. Das Package enthält keine Node oder Librabries.

## Messages 


### PointOfInterest


Die PointOfInterest-Message enthält alle Informationen, um einen POI zu erstellen, zu verschicken und zu speichern.

~~~~~~ {.cpp}
PointOfInterest.msg



int32 DEFAULT=100
int32 VICTIM=200
int32 OBJECT=300
int32 GRIPPABLE_OBJECT=400
int32 PERSON=600
int32 ROOMBA=700
int32 HAZARD_MATERIAL=800
int32 START_POSITION=900
int32 START_ORIENTATION=1000

int32 type
string name
string remarks

geometry_msgs/Pose pose
~~~~~~

* `type` bezeichnet den Typ des POIs. Es kann eine der in dieser Message vorhandenen Konstanten verwendet werden. 
* `name` bezeichnet den Namen des POIs. Dieser Name muss einzigartig sein, da die POIs über ihren Namen unterschieden werden.
* `remarks`: Hier können Anmerkungen reingeschrieben werden. Diese werden in der GUI angezeigt.
* `pose` bezeichnet die Position und Orientierung des POIS im /map-Frame.


### ModifyPOI


ModifyPOI ist dafür zuständig, einen vorhandenen POI zu verändern.

~~~~~~ {.cpp}
ModifyPOI.msg

PointOfInterest poi
string old_name
~~~~~~

* `poi` beinhaltet den POI, durch den der alte ersetzt werden soll.
* `old_name` bezeichnet den Namen des POIs, der verändert werden soll.


### TargetUnreachable



TargetUnreachable wird von der Navigation versendet, sobald kein Pfad mehr zu einem Ziel geplant werden kann.


~~~~~~ {.cpp}
TargetUnreachable.msg

int8 UNKNOWN=0
int8 TILT_OCCURED=10
int8 GRAVE_TILT_OCCURED=15
int8 STALL_OCCURED=20
int8 LASER_OBSTACLE=30

int8 reason
~~~~~~

* `reason` kann einen von den in dieser Message definierten Konstanten annehmen und beschreibt den Grund des Fehlers.


### SaveMap

SaveMap wird versendet, wenn eine Karte gespeichert oder geladen werden soll und beinhaltet den Dateipfad zum Kartenordner.


~~~~~~ {.cpp}
SaveMap.msg

string filename
~~~~~~

* `filename` bezeichnet den Dateipfad zum Kartenordner.


### PointsOfInterest


PointsOfInterest wird verwendet, um alle aktuellen POIs zu versenden.


~~~~~~ {.cpp}
PointsOfInterest.msg

PointOfInterest[] pois
~~~~~~

* `pois` beinhaltet einen Vektor mit allen aktuellen POIs.


### StartNavigation


StartNavigation ist eine von zwei Methoden, um eine Navigation zu starten. Hier wird Der POI mitgegeben, zu dem der Roboter navigieren soll.


~~~~~~ {.cpp}
StartNavigation.msg

geometry_msgs/Pose goal
float32 distance_to_target
bool skip_final_turn
bool fast_planning
~~~~~~

* `goal` beinhaltet den Ziel-POI
* `distance_to_target`: Hier kann angegeben werden, ab welcher Distanz zum Ziel die Navigation als erfolgreich abgeschlossen wird. 
* `skip_final_turn`: Hier kann eingestellt werden, ob der Roboter sich am Ziel-POI in Richtund der POI-Orientierung ausrichten soll oder nicht.
* `fast_planning`: Mit dieser Option kann ein experimentelles "Schnelles Planen" eingeschaltet werden. Es werden nur Pfade geplant, die sich in einer Boundingbox zwischen Roboter und Zielposition befinden.


### MapLayers

MapLayers definiert die vorhanden Kartenebenen als Konstanten und kann zudem verwendet werden, um einzelne Ebenen ein- oder auszuschalten.


~~~~~~ {.cpp}
MapLayers.msg

int32 SLAM_LAYER=0
int32 MASKING_LAYER=1
int32 KINECT_LAYER=2
int32 SICK_LAYER=3

int32 layer
bool state
~~~~~~

* `layer` enthält die Kartenebenen-ID und kann einen Wert dern in dieser Message definierten Konstanten annehmen.
* `state` besagt, ob die ausgewählte Kartenebene aktiviert sein soll.




### NavigateToPOI


NavigateToPOI ist die zweite Art eine Navigation zu starten. Anstatt das gesamte POI-Objekt mitzugeben, wird nur der Name eines bereits im map_manger vorhandenen POIs mitgegeben, der daraufhin von der Navigation nachgeschlagen wird.

~~~~~~ {.cpp}
NavigateToPOI.msg

string poi_name
float32 distance_to_target
bool skip_final_turn
~~~~~~

* `poi_name` beschreibt den Namen des POIS, zu dem navigiert werden soll.
* `distance_to_target` siehe StartNavigation
* `skip_final_turn` siehe StartNavigation


### ModifyMap


Mit dieser Message können Bereiche in einzelnen Kartenebenen verändert werden.


~~~~~~ {.cpp}
ModifyMap.msg

int32 FREE = 0         
int32 BLOCKED = 100    
int32 OBSTACLE = 99     
int32 NOT_MASKED = -1 

geometry_msgs/Point[] region
int32 maskAction
int32 mapLayer 
~~~~~~

* `region` beschreibt die Eckpunkte des Polygons, das maskiert werden soll.
* `maskAction` kann einen Wert der in dieser Message definierten Konstanten annehmen. OBSTACLE  wird in der Karte rot dargestellt. Mit NOT_MASKED können bereits maskierte Bereiche wieder gelöscht werden.
* `mapLayer` enthält die ID der zu verändernden Kartenebene. Die IDs sind in der Message MapLayers definiert.



### DeletePointOfInterest

Löscht einen vorhandenen POI.


~~~~~~ {.cpp}
DeletePointOfInterest.msg

string name
~~~~~~

* `name` beschreibt den Namen des zu löschenden POIs.


### DoMapping



Mit dieser Message kann das Mapping ein- oder ausgeschaltet werden.

~~~~~~ {.cpp}
DoMapping.msg

bool state
~~~~~~

* `state` beinhaltet den Zustand des Mappings (true = an, false = off).


## Services 

### GetPointsOfInterest


Über diesen Service kann die aktuelle Liste der POIs angefordert werden.


~~~~~~ {.cpp}
GetPointsOfInterest.srv

---
PointsOfInterest poi_list
~~~~~~

* `poi_list` beinhaltet einen Vektor mit allen aktuellen POIs.


