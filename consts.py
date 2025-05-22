# GRUNNLEGGENDE
# Skjermen e kvadratisk, med denne piksel størrelsen. 
SCREEN_WIDTH = 960 # Kunne hatt 1240 men det bli så unødvendig bredt
SCREEN_HEIGHT = 700

# Kor mange piksla en meter skal dekke. 
MAP_SCALE = 100

# Kor mang meter i sekundet punktet vi følge skal beveg seg
ROBOT_SPEED = 0.1

# Hvor høy oppløsning vi skal ha på grid strukturen
GRID_SLOTS_PER_METER = 8


# LOCALIZATION
# Hvor langt det kan vær mellom punkt for at localization skal fortsatt matche dem. 
LOCALIZATION_POINT_MATCH_DISTANCE = 0.10


# SPLIT AND MERGE
# Hvor nøye split and merge skal være, alstå distanse fra linja som skaper en ny split. 
# Må ha en lav nok verdi her for at vi ska få med oss alle hjørner og slik unngå linjer som 
# som ikkje er linjer. 
SPLIT_DISTANCE = 0.02

# Parameters for kart vedlikeholdt
# How many points we require per meter to classify something as a line. 
# This would let the Limo recognize a flat wall of 1 meter, from 5 meters away, which semms quite sufficient.
LINE_POINTS_PER_METER = 5

# The minimum line length, lines shorter than this are removed. 
LINE_MIN_LENGTH = 0.10

# # Lengste avstanden mellom punkt på en linje skal maks vær n ganger den korteste avstanden. 
# LINE_POINT_DISTANCE_VARIANCE = 30

# Ingen punkt i en linje skal vær lenger fra neste punkt enn dette. 
LINE_POINT_MAX_JUMP = 0.2


# KART VEDLIKEHOLD
# Når et punkt skal slås sammen med et anna punkt
POINT_MERGE_DISTANCE = 0.05

# Kor mang prosent mot det nye punktet punktet på kartet ska flytt seg, mellom 0 og 1. 
# Vi velge oss en verdi under 50% her for at om den går seg vill en iterasjon skal det ikkje permanent shift heile mappet. 
POINT_MERGE_MOVEMENT = 0.05

# Når et punkt skal slås inn i linja
POINT_LINE_MERGE_DISTANCE = POINT_MERGE_DISTANCE

# Hvor langt gjennom en linje vi må sjå for at det skal fjern linja
SEE_THROUGH_LINE_BONUS = 0.05

# Kor mange rays jevnt fordelt gjennom scannen, med en random offset pr runde, som fjerne eksisterende linjer. 
SEE_THROUGH_LINE_COUNT = 3

# Begrens bredden av frontiers
MIN_FRONTIER_WIDTH = 0.4
MAX_FRONTIER_WIDTH = 2

# Vi begrense kor langt unna vi kan sjå for å begrens unøyaktigheten
SIGHT_DISTANCE = 4